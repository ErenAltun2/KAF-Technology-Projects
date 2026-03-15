# main_controller.py
import cv2
import time
import threading
from picamera2 import Picamera2
from dronekit import connect
import detect_tracking5
import kutu_acma
import pence
import stream_server
class DroneVisionController:
    def __init__(self):
        print("🚁 Drone Vision Controller başlatılıyor...")
        
        # Pixhawk bağlantısı
        try:
            print("📡 Pixhawk'a bağlanılıyor...")
            self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)
            print("✅ Pixhawk bağlantısı OK")
        except Exception as e:
            print(f"❌ Pixhawk bağlantı hatası: {e}")
            self.vehicle = None
        
        # Kamera kurulumu
        try:
            print("📹 Kamera başlatılıyor...")
            self.camera = Picamera2()
            self.camera.preview_configuration.main.size = (640, 480)
            self.camera.preview_configuration.main.format = "RGB888"
            self.camera.configure("preview")
            self.camera.start()
            time.sleep(2)  # Kameraya zaman ver
            print("✅ Kamera başlatıldı")
        except Exception as e:
            print(f"❌ Kamera başlatma hatası: {e}")
            return
        
        # Web server başlat
        print("🌐 Web server başlatılıyor...")
        stream_server.start_server(host="0.0.0.0", port=5001)
        print("✅ Web server başlatıldı: http://localhost:5001/video")
        
        # Kontrol değişkenleri
        self.last_detection_time = {}  # Her şekil için son tespit zamanı
        self.cooldown_period = 3.0  # Aynı şekil için 3 saniye bekleme süresi
        self.running = True
        
        # İstatistikler
        self.stats = {
            'triangle_count': 0,
            'hexagon_count': 0,
            'total_frames': 0
        }
    
    def execute_servo_action(self, target_type, center):
        """Tespit edilen şekle göre servo aksiyonu gerçekleştir"""
        if not self.vehicle:
            print("⚠️ Pixhawk bağlantısı yok - servo aksiyonu atlanıyor")
            return
        
        current_time = time.time()
        
        # Cooldown kontrolü
        if target_type in self.last_detection_time:
            time_since_last = current_time - self.last_detection_time[target_type]
            if time_since_last < self.cooldown_period:
                return  # Çok erken, aksiyonu atla
        
        # Son tespit zamanını güncelle
        self.last_detection_time[target_type] = current_time
        
        try:
            if target_type == "triangle":
                print("🔺 KIRMIZI ÜÇGEN TESPİT EDİLDİ - Kutu açma/kapama başlatılıyor...")
                self.stats['triangle_count'] += 1
                
                # Kutu açma sekansı
                print("📦 Kutu açılıyor...")
                kutu_acma.box_open(self.vehicle, channel=9, degree=180, wait=1.5)
                
                print("📦 Kutu kapatılıyor...")
                kutu_acma.box_close(self.vehicle, channel=9, degree=0, wait=1.5)
                
                print("✅ Kutu sekansı tamamlandı")
                
            elif target_type == "hexagon":
                print("🔵 MAVİ ALTIGEN TESPİT EDİLDİ - Pençe açma/kapama başlatılıyor...")
                self.stats['hexagon_count'] += 1
                
                # Pençe açma sekansı
                print("🤏 Pençe açılıyor...")
                kutu_acma.box_open(self.vehicle, channel=10, degree=180, wait=1.5)
                
                print("🤏 Pençe kapatılıyor...")
                kutu_acma.box_close(self.vehicle, channel=10, degree=0, wait=1.5)
                
                # Pençe rotasyon sekansı
                print("✅ Pençe sekansı tamamlandı")
                
        except Exception as e:
            print(f"❌ Servo aksiyonu hatası: {e}")
    
    def print_stats(self):
        """İstatistikleri yazdır"""
        print(f"\n📊 İSTATİSTİKLER:")
        print(f"   🔺 Kırmızı üçgen tespiti: {self.stats['triangle_count']}")
        print(f"   🔵 Mavi altigen tespiti: {self.stats['hexagon_count']}")
        print(f"   📷 İşlenen frame sayısı: {self.stats['total_frames']}")
        print(f"   🌐 Web stream: http://localhost:5001/video")
    
    def run(self):
        """Ana döngü - kamera okuma ve tespit"""
        print("\n🎯 Tespit sistemi çalışıyor...")
        print("📱 Web stream: http://localhost:5001/video")
        print("⌨️  Durdurmak için 'q' tuşuna basın\n")
        
        frame_count = 0
        last_stats_time = time.time()
        
        try:
            while self.running:
                try:
                    # Frame al
                    frame = self.camera.capture_array()
                    rgb_image = frame.copy()  # RGB'den BGR'ye dönüşüm yok, direkt kullan
                    
                    # Tespit yap
                    vis_frame, target_type, center = detect_tracking5.detect_target_and_get_output(
                        rgb_image, search_for=None
                    )
                    
                    # Web stream'e frame gönder
                    stream_server.update_frame(vis_frame)
                    
                    # Eğer tespit varsa servo aksiyonu
                    if target_type and center:
                        # Arka planda servo aksiyonu gerçekleştir
                        action_thread = threading.Thread(
                            target=self.execute_servo_action, 
                            args=(target_type, center)
                        )
                        action_thread.daemon = True
                        action_thread.start()
                    
                    # Frame sayacı
                    frame_count += 1
                    self.stats['total_frames'] = frame_count
                    
                    # Her 5 saniyede bir istatistik yazdır
                    current_time = time.time()
                    if current_time - last_stats_time >= 5.0:
                        self.print_stats()
                        last_stats_time = current_time
                    
                    # CPU'ya biraz nefes ver
                    time.sleep(0.033)  # ~30 FPS
                    
                except KeyboardInterrupt:
                    print("\n⏹️ Kullanıcı tarafından durduruldu")
                    break
                except Exception as e:
                    print(f"⚠️ Frame işleme hatası: {e}")
                    time.sleep(0.1)
                    continue
                    
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        print("\n🧹 Temizlik yapılıyor...")
        
        self.running = False
        
        if hasattr(self, 'camera'):
            try:
                self.camera.stop()
                print("✅ Kamera durduruldu")
            except:
                pass
        
        if self.vehicle:
            try:
                self.vehicle.close()
                print("✅ Pixhawk bağlantısı kapatıldı")
            except:
                pass
        
        # Son istatistikleri göster
        self.print_stats()
        print("\n👋 Program sonlandı.")

def main():
    """Ana fonksiyon"""
    print("🚁 Drone Vision Controller")
    print("=" * 50)
    
    try:
        controller = DroneVisionController()
        controller.run()
    except KeyboardInterrupt:
        print("\n⏹️ Program kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"❌ Genel hata: {e}")

if __name__ == "__main__":
    main()
