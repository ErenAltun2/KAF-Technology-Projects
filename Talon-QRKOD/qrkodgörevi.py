#!/usr/bin/env python3
from pymavlink import mavutil
import time
import threading
import math
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar  # QR kod okuma için
import numpy as np
from geopy.distance import geodesic  # Yeni eklendi: coğrafi hesaplamalar için
from geopy.point import Point  # Yeni eklendi

qr_scan_active = True
qr_scan_lock = threading.Lock()
current_agl = 0.0
current_airspeed = 0.0
agl_updated = 0.0
running = True  #
master = None

# ROS2 Kamera Abonesi ve QR Kod Okuyucu
class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/world/talon_runway/model/mini_talon_vtail/link/base_link/sensor/talon_camera/image',
            self.image_callback,
            10
        )
        self.msg_count = 0
        self.timer = self.create_timer(2.0, self.check_status)
        self.qr_detected = False
        self.qr_data = ""
        self.qr_history = []  # QR geçmişi
        print("📷 Kamera abonesi başlatıldı")
    
    def image_callback(self, msg):
        try:
            self.msg_count += 1
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # QR tarama aktifse tarama yap
            global qr_scan_active
            if qr_scan_active:
                cv_image = self.scan_qr_code(cv_image)
            
            # Görüntüyü göster
            cv2.imshow("Mini Talon Camera -KAF TECHNOLOGY-QR Scanner", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')
    
# Değiştirilecek kısım: scan_qr_code fonksiyonu
    def scan_qr_code(self, image):
        """Görüntüde QR kodu tarar ve görselleştirir"""
        try:
            # Görüntü ön işleme - QR okunabilirliğini artır
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Histogram eşitleme (düşük ışık için)
            gray = cv2.equalizeHist(gray)
            # Kenar keskinleştirme
            kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            sharpened = cv2.filter2D(gray, -1, kernel)
            
            # QR kodları tara (hem renkli hem gri skala)
            decoded_objects = pyzbar.decode(image) + pyzbar.decode(sharpened)
            
            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                
                # QR boyutu kontrolü (çok küçükse yoksay)
                x, y, w, h = obj.rect
                min_qr_size = 50  # Minimum kabul edilebilir QR boyutu (piksel)
                if w < min_qr_size or h < min_qr_size:
                    self.get_logger().info(f"Küçük QR göz ardı: {w}x{h}px (Min: {min_qr_size}px)")
                    continue
                    
                # Yeni QR kodu ise kaydet
                if qr_data not in self.qr_history:
                    self.qr_history.append(qr_data)
                    self.qr_data = qr_data
                    self.qr_detected = True
                    print(f"✅ YENİ QR Kodu Okundu: {qr_data}")
                    print(f"📋 Toplam QR: {len(self.qr_history)}")
                    # ROS loguna da yaz
                    self.get_logger().info(f"Yeni QR: {qr_data} ({w}x{h}px)")
                
                # QR kodun etrafına çerçeve çiz
                points = obj.polygon
                if len(points) > 4:
                    hull = cv2.convexHull(
                        np.array([point for point in points], dtype=np.float32)
                    )
                    hull = list(map(tuple, np.squeeze(hull)))
                else:
                    hull = points
                
                n = len(hull)
                for j in range(0, n):
                    cv2.line(image, hull[j], hull[(j+1) % n], (0, 0, 255), 3)
                
                # Bilgi metni
                cv2.putText(image, qr_data, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 0), 2)
                cv2.putText(image, f"{w}x{h}px", (x, y+h+20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
                
            # QR kodu sayısını göster
            cv2.putText(image, f"QR KOD: {len(self.qr_history)}", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # QR tarama aktif olduğunu göster
            status_text = "QR TARAMA AKTIF" if qr_scan_active else "QR TARAMA PASIF"
            color = (0, 255, 0) if qr_scan_active else (0, 0, 255)
            cv2.putText(image, status_text, (10, image.shape[0]-20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            return image
            
        except Exception as e:
            self.get_logger().error(f'QR tarama hatası: {str(e)}')
            return image
    def check_status(self):
        if self.msg_count == 0:
            self.get_logger().warn('Henüz kamera mesajı alınmadı')
        else:
            self.get_logger().info(f'Kamera mesajları alınıyor: {self.msg_count} mesaj')
            if len(self.qr_history) > 0:
                self.get_logger().info(f'QR Geçmişi: {self.qr_history}')

# ROS2'yi başlatan fonksiyon
def start_ros2_camera():
    """ROS2 kamera thread'ini başlatır"""
    def ros2_thread():
        try:
            print("🚀 ROS2 kamera thread'i başlatılıyor...")
            rclpy.init()
            camera_subscriber = CameraSubscriber()
            print("📷 Kamera subscriber oluşturuldu")
            rclpy.spin(camera_subscriber)
        except Exception as e:
            print(f"❌ ROS2 kamera hatası: {e}")
        finally:
            try:
                camera_subscriber.destroy_node()
                rclpy.shutdown()
                cv2.destroyAllWindows()
                print("🔄 ROS2 kamera kapatıldı")
            except:
                pass
    
    ros2_camera_thread = threading.Thread(target=ros2_thread, daemon=True)
    ros2_camera_thread.start()
    print("✅ ROS2 kamera thread'i başlatıldı")
    return ros2_camera_thread

with qr_scan_lock:
    qr_scan_active = True
    print("🔍 QR tarama aktif edildi (uçuş boyunca)")

# ROS2 kamera sistemini başlat
print("🎥 ROS2 kamera sistemi başlatılıyor...")
camera_subscriber_instance = None
try:
    camera_thread = start_ros2_camera()
    time.sleep(3)  # ROS2'nin başlaması için bekle
    print("📹 Kamera sistemi hazır")
except Exception as e:
    print(f"⚠️ ROS2 kamera başlatılamadı: {e}")
    print("📝 Kamera olmadan devam ediliyor...")

# Yükseklik ve hız okuma thread'i
def telemetry_reader():
    global current_agl, current_airspeed, agl_updated, running, master  # <--- DÜZELTİLDİ
    
    while running:
        try:
            # GLOBAL_POSITION_INT mesajını al
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if msg:
                current_agl = msg.relative_alt / 1000.0  # mm to meters
                agl_updated = time.time()
            
            # VFR_HUD mesajını al (airspeed için)
            msg2 = master.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
            if msg2:
                current_airspeed = msg2.airspeed
                
        except Exception as e:
            print(f"⚠️ Telemetri okuma hatası: {e}")
        time.sleep(0.01)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """İki koordinat arasında yön açısı (bearing) hesapla"""
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    diff_lon = math.radians(lon2 - lon1)
    
    x = math.sin(diff_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diff_lon))
    initial_bearing = math.atan2(x, y)
    
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    
    return compass_bearing


class TalonCameraNavigator:
    def __init__(self, connection_string='udp:127.0.0.1:14550'):
        self.connection_string = connection_string
        self.master = None
        global master  # <--- Global master referansını al
        master = self.master  # <--- Eşitleme yapı
        
        # Hedef koordinatlar
        self.target_lat = -35.3614547
        self.target_lon = 149.1652376
        self.target_alt = 10.0  # metre (AGL)
        
        # Telemetri verileri
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_agl = 0.0
        self.current_airspeed = 0.0
        self.running = True

        # QR dalış parametreleri
        self.dive_start_distance = 55.0  # QR'dan 50m geride dalış başlat
        self.dive_start_altitude = 40.0  # 12m irtifada dalış başlat
        self.qr_approach_altitude = 5.0  # QR'a 5m yüksekten yaklaş
        self.qr_horizontal_distance = 5.0  # QR'dan 5m yatay mesafe
        self.dive_angle_target = 45.0  # 45° dalış açısı
        
        # Kamera yaklaşma parametreleri
        self.camera_approach_distance = 60.0  # 20 metre yaklaşınca kamera manevrası
        self.camera_approach_alt = 5.0  # Kamera yaklaşma yüksekliği
        
        # Ara waypoint için bayrak
        self.intermediate_target_set = False
        
    def connect(self):
        """MAVLink bağlantısını kur"""
        try:
            self.master = mavutil.mavlink_connection(self.connection_string)
            global master  # <--- Global değişkeni güncelle
            master = self.master  # <--- BU SATIRI EKLEYİN
            self.master.wait_heartbeat()
            print("✅ Bağlantı kuruldu.")
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def telemetry_reader(self):
        """Telemetri okuma thread'i"""
        while self.running:
            try:
                # GLOBAL_POSITION_INT mesajını al
                msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
                if msg:
                    self.current_lat = msg.lat / 1e7
                    self.current_lon = msg.lon / 1e7
                    self.current_agl = msg.relative_alt / 1000.0  # mm to meters
                
                # VFR_HUD mesajını al (airspeed için)
                msg2 = self.master.recv_match(type='VFR_HUD', blocking=False, timeout=0.1)
                if msg2:
                    self.current_airspeed = msg2.airspeed
                    
            except Exception as e:
                print(f"⚠️ Telemetri okuma hatası: {e}")
            time.sleep(0.01)
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """İki koordinat arasındaki mesafeyi hesapla (metre)"""
        R = 6371000  # Dünya yarıçapı (metre)
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) * math.sin(delta_lat / 2) +
             math.cos(lat1_rad) * math.cos(lat2_rad) *
             math.sin(delta_lon / 2) * math.sin(delta_lon / 2))
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        
        return distance

    def calculate_dive_angle(self, horizontal_distance, altitude_diff):
        """Dalış açısını hesaplar"""
        if horizontal_distance <= 0:
            return 0
        angle = math.degrees(math.atan2(altitude_diff, horizontal_distance))
        return angle

    def check_dive_conditions(self):
        """40x40 koşuluna göre QR dalış koşullarını kontrol eder"""
        distance = self.calculate_distance(
            self.current_lat, self.current_lon,
            self.target_lat, self.target_lon
        )
        
        altitude_diff = self.current_agl - self.qr_approach_altitude
        dive_angle = self.calculate_dive_angle(distance, altitude_diff)
        
        # 40x40 KOŞULU: Tam 40m mesafe ve 40m irtifa
        TARGET_DISTANCE = 40.0  # 40 metre mesafe
        TARGET_ALTITUDE = 40.0   # 40 metre irtifa
        DISTANCE_TOLERANCE = 10.0  # ±5m tolerans
        ALTITUDE_TOLERANCE = 10.0  # ±5m tolerans
        
        conditions = {
            # 40x40 koşulu - sıkı kontrol
            'distance_40m': abs(distance - TARGET_DISTANCE) <= DISTANCE_TOLERANCE,
            'altitude_40m': abs(self.current_agl - TARGET_ALTITUDE) <= ALTITUDE_TOLERANCE,
            
            # Dalış açısı kontrolü (45° hedef)
            'angle_ok': 30 <= dive_angle <= 60,
            
            # Hız kontrolü
            'speed_ok': 10 <= self.current_airspeed <= 18,
            
            # Pozisyon güvenlik kontrolü
            'position_safe': distance >= 10,  # En az 5m mesafe
            
            # Sensör kontrolleri
            'sensors_ok': (
                self.current_agl > 0 and 
                self.current_airspeed > 0 and 
                abs(self.current_lat) > 0.001 and 
                abs(self.current_lon) > 0.001
            )
        }
        
        # ANA KOŞUL: 40x40 ve temel güvenlik
        main_condition = (
            conditions['distance_40m'] and 
            conditions['altitude_40m'] and
            conditions['sensors_ok'] and
            conditions['position_safe']
        )
        
        # FBWA moda geçme koşulu
        fbwa_ready = main_condition and conditions['speed_ok']
        
        # 45° dalış başlatma koşulu
        dive_ready = fbwa_ready and conditions['angle_ok']
        
        if not main_condition:
            print("❌ 40x40 Dalış koşulları:")
            print(f"   📏 Mesafe: {distance:.1f}m (Hedef: {TARGET_DISTANCE}±{DISTANCE_TOLERANCE}m) {'✅' if conditions['distance_40m'] else '❌'}")
            print(f"   📊 İrtifa: {self.current_agl:.1f}m (Hedef: {TARGET_ALTITUDE}±{ALTITUDE_TOLERANCE}m) {'✅' if conditions['altitude_40m'] else '❌'}")
            print(f"   🏃 Hız: {self.current_airspeed:.1f}m/s {'✅' if conditions['speed_ok'] else '❌'}")
            print(f"   📐 Dalış açısı: {dive_angle:.1f}° {'✅' if conditions['angle_ok'] else '❌'}")
            print(f"   🔧 Sensörler: {'✅' if conditions['sensors_ok'] else '❌'}")
            print(f"   🛡️ Güvenlik: {'✅' if conditions['position_safe'] else '❌'}")
            print(f"   🎯 40x40 Koşul: {'✅' if main_condition else '❌'}")
            print(f"   🎮 FBWA Hazır: {'✅' if fbwa_ready else '❌'}")
            print(f"   📉 Dalış Hazır: {'✅' if dive_ready else '❌'}")
        
        return {
            'main_condition': main_condition,
            'fbwa_ready': fbwa_ready,
            'dive_ready': dive_ready,
            'conditions': conditions,
            'distance': distance,
            'altitude': self.current_agl,
            'dive_angle': dive_angle
        }

        
    def calculate_dive_start_point(self, drone_pos, qr_pos):
        """
        Dalış başlangıç noktasını hesaplar - İyileştirilmiş versiyon
        """
        try:
            drone_lat, drone_lon = drone_pos
            qr_lat, qr_lon = qr_pos

            # Mevcut mesafeyi kontrol et
            current_distance = self.calculate_distance(drone_lat, drone_lon, qr_lat, qr_lon)
            
            print(f"📏 Mevcut QR mesafesi: {current_distance:.1f}m")
            
            # Eğer çok yakınsa, daha uzak bir waypoint hesapla
            if current_distance < self.dive_start_distance:
                print(f"⚠️ Çok yakın! Waypoint mesafesi artırılıyor...")
                adjusted_distance = self.dive_start_distance + 20  # 20m ekstra
            else:
                adjusted_distance = self.dive_start_distance

            # Uçağın QR'a olan yönünü hesapla
            bearing_drone_to_qr = calculate_bearing(drone_lat, drone_lon, qr_lat, qr_lon)
            
            print(f"📐 Uçak -> QR yönü: {bearing_drone_to_qr:.1f}°")
            
            # QR'dan uçağa doğru (ters yön) belirli mesafede waypoint oluştur
            dive_start_point = geodesic(meters=adjusted_distance).destination(
                point=Point(qr_lat, qr_lon),
                bearing=(bearing_drone_to_qr + 180) % 360
            )
            
            waypoint_lat, waypoint_lon = dive_start_point.latitude, dive_start_point.longitude
            
            # Doğrulama kontrolleri
            distance_drone_to_waypoint = self.calculate_distance(
                drone_lat, drone_lon, waypoint_lat, waypoint_lon
            )
            distance_waypoint_to_qr = self.calculate_distance(
                waypoint_lat, waypoint_lon, qr_lat, qr_lon
            )
            
            print(f"📊 Waypoint doğrulama:")
            print(f"   📍 Waypoint: {waypoint_lat:.6f}, {waypoint_lon:.6f}")
            print(f"   📏 Uçak -> Waypoint: {distance_drone_to_waypoint:.1f}m")
            print(f"   📏 Waypoint -> QR: {distance_waypoint_to_qr:.1f}m")
            print(f"   📊 Hedef irtifa: {self.dive_start_altitude}m")
            
            # Waypoint çok yakın veya çok uzaksa ayarla
            if distance_drone_to_waypoint < 20:
                print("⚠️ Waypoint çok yakın, ayarlanıyor...")
                # Biraz daha uzak bir nokta hesapla
                dive_start_point = geodesic(meters=adjusted_distance + 30).destination(
                    point=Point(qr_lat, qr_lon),
                    bearing=(bearing_drone_to_qr + 180) % 360
                )
                waypoint_lat, waypoint_lon = dive_start_point.latitude, dive_start_point.longitude
                
            elif distance_drone_to_waypoint > 200:
                print("⚠️ Waypoint çok uzak, ayarlanıyor...")
                # Biraz daha yakın bir nokta hesapla
                dive_start_point = geodesic(meters=adjusted_distance - 20).destination(
                    point=Point(qr_lat, qr_lon),
                    bearing=(bearing_drone_to_qr + 180) % 360
                )
                waypoint_lat, waypoint_lon = dive_start_point.latitude, dive_start_point.longitude

            print(f"✅ Geopy waypoint hazır!")
            return (waypoint_lat, waypoint_lon, self.dive_start_altitude)

        except Exception as e:
            print(f"❌ Geopy waypoint hesaplama hatası: {e}")
            return None

    def set_mode(self, mode):
        """Uçuş modunu değiştir"""
        mode_mapping = {
            'AUTO': 10,
            'LOITER': 12,
            'RTL': 11,
            'FBWA': 5,
            'MANUAL': 0
        }
        
        if mode not in mode_mapping:
            print(f"❌ Geçersiz mod: {mode}")
            return False
        
        # Mod değiştirme komutu gönder
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode]
        )
        
        # Mod değişikliğini doğrula
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg and msg.custom_mode == mode_mapping[mode]:
                print(f"✅ {mode} moduna geçildi.")
                return True
        
        print(f"❌ {mode} moduna geçilemedi.")
        return False
    
    def send_waypoint(self):
        """Waypoint gönder - Geliştirilmiş versiyon"""
        print(f"📍 Waypoint gönderiliyor: {self.target_lat:.6f}, {self.target_lon:.6f}, {self.target_alt}m")
        
        try:
            # Mission clear
            self.master.mav.mission_clear_all_send(
                self.master.target_system,
                self.master.target_component
            )
            time.sleep(1)  # Daha uzun bekleme
            
            # Mission count gönder (1 waypoint)
            self.master.mav.mission_count_send(
                self.master.target_system,
                self.master.target_component,
                1
            )
            
            # Mission request bekle
            msg = self.master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=10)
            if msg:
                print(f"✅ Mission request alındı: seq={msg.seq}")
                
                # Waypoint item gönder
                self.master.mav.mission_item_send(
                    self.master.target_system,
                    self.master.target_component,
                    0,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0,  # current
                    1,  # autocontinue
                    0,  # param1 (hold time)
                    3,  # param2 (acceptance radius) - biraz artırıldı
                    0,  # param3
                    0,  # param4 (yaw angle)
                    self.target_lat,
                    self.target_lon,
                    self.target_alt
                )
                print(f"📍 Waypoint item gönderildi")
            else:
                print("❌ Mission request alınamadı!")
                return False
        
            # Mission ACK bekle
            msg = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            if msg:
                success = msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                if success:
                    print("✅ Waypoint başarıyla kabul edildi.")
                else:
                    print(f"❌ Waypoint reddedildi: {msg.type}")
                return success
            else:
                print("❌ Mission ACK alınamadı!")
                return False
        
        except Exception as e:
            print(f"❌ Waypoint gönderme hatası: {e}")
            return False
    
    def qr_dive_maneuver(self):
        """QR dalış manevrası (45° eğimle)"""
        print("🎯 QR dalış manevrası başlatılıyor...")
        
        # QR taramayı aktif et
        global qr_scan_active
        with qr_scan_lock:
            qr_scan_active = True
            print("🔍 QR tarama aktif edildi")
        
        # FBWA moduna geç
        if not self.set_mode('FBWA'):
            print("❌ FBWA moduna geçilemedi")
            return False
        
        # QR dalış koşullarını kontrol et
        dive_result = self.check_dive_conditions()
        if not dive_result['dive_ready']:
            print(f"❌ Dalış koşulları sağlanmıyor: {dive_result['conditions']}")
            return False
        
        print("✅ Dalış koşulları sağlandı, 45° dalış başlatılıyor...")
        
        # 45° dalış manevrası
        dive_start_time = time.time()
        dive_duration = 15  # 15 saniye maksimum dalış
        qr_detected = False
        
        while (time.time() - dive_start_time) < dive_duration and not qr_detected:
            # QR'a olan mesafe ve açı hesapla
            distance = self.calculate_distance(
                self.current_lat, self.current_lon,
                self.target_lat, self.target_lon
            )
            
            altitude_diff = self.current_agl - self.qr_approach_altitude
            current_angle = self.calculate_dive_angle(distance, altitude_diff)
            
            # 45° dalış için RC komutları
            elevator = 1200  # Sabit 45° burun aşağı
            throttle = 1400  # Sabit dalış throttle
            # Hız bazlı throttle kontrolü sabit kalsın
            
            # RC komutlarını gönder
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                1500,     # Aileron (düz)
                throttle, # Throttle
                elevator, # Elevator (dalış)
                1500,     # Rudder (düz)
                0, 0, 0, 0
            )
            
            print(f"🎯 Dalış - Mesafe: {distance:.1f}m, Açı: {current_angle:.1f}°, "
                f"AGL: {self.current_agl:.1f}m, Hız: {self.current_airspeed:.1f}m/s")
            
            # QR mesafesine ulaştıysa
            if distance <= self.qr_horizontal_distance + 3 and 3 <= self.current_agl <= 7:                
                print("✅ QR aralığına ulaşıldı!")
                qr_detected = True
                break
            
            # Çok alçaksa güvenlik
            if self.current_agl < 2:
                print("⚠️ Çok alçak! Dalış durduruldu.")
                break
            
            time.sleep(0.1)
        
        # Pull-up (toparlanma) manevrası
        print("📈 Pull-up manevrası başlatılıyor...")
        pullup_start_time = time.time()
        
        while (time.time() - pullup_start_time) < 8:
            # G kuvveti hesapla (güvenli pull-up için)
            turn_radius = 50  # 50m dönüş yarıçapı
            g_force = (self.current_airspeed ** 2) / (turn_radius * 9.81) + 1
            
            if g_force > 2.5:  # Çok yüksek G, yumuşat
                elevator = 1600  # Daha yumuşak pull-up
                throttle = 1700
            else:  # Normal pull-up
                elevator = 1700  # Güçlü pull-up
                throttle = 1800
            
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                1500, throttle, elevator, 1500, 0, 0, 0, 0
            )
            
            print(f"📈 Pull-up - AGL: {self.current_agl:.1f}m, Hız: {self.current_airspeed:.1f}m/s, G: {g_force:.2f}")
            
            # 15m irtifaya ulaştıysa pull-up bitir
            if self.current_agl >= 15:
                break
            
            time.sleep(0.1)
        
        # RC override temizle
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        print("🔄 RC override temizlendi")
        print("✅ QR dalış manevrası tamamlandı!")
        
        return True
    
    def navigate_to_waypoint(self):
        """Ana navigasyon fonksiyonu - İlk waypoint olarak Geopy waypoint"""
        print("🚀 QR dalış navigasyonu başlatılıyor...")
        
        # Bağlantı kur
        if not self.connect():
            return False
        
        # Telemetri thread'ini başlat
        telemetry_thread = threading.Thread(target=self.telemetry_reader, daemon=True)
        telemetry_thread.start()
        
        # İlk pozisyonu bekle
        print("📡 İlk telemetri verileri bekleniyor...")
        position_timeout = 10
        start_wait = time.time()
        
        while (time.time() - start_wait) < position_timeout:
            if abs(self.current_lat) > 0.001 and abs(self.current_lon) > 0.001:
                print(f"✅ İlk pozisyon alındı: {self.current_lat:.6f}, {self.current_lon:.6f}")
                break
            time.sleep(0.5)
        else:
            print("❌ İlk pozisyon alınamadı!")
            return False
        
        # ÖNEMLİ: Orijinal QR hedefini kaydet
        original_qr_lat = self.target_lat
        original_qr_lon = self.target_lon
        original_qr_alt = self.target_alt
        
        # İLK WAYPOINT OLARAK GEOPY WAYPOINT HESAPLA
        print("🎯 İlk waypoint olarak Geopy waypoint hesaplanıyor...")
        
        dive_start_coords = self.calculate_dive_start_point(
            (self.current_lat, self.current_lon),
            (original_qr_lat, original_qr_lon)
        )
        
        if not dive_start_coords:
            print("❌ Geopy waypoint hesaplanamadı")
            return False
        
        # Geopy waypoint'i ilk hedef yap
        self.target_lat = dive_start_coords[0]
        self.target_lon = dive_start_coords[1]
        self.target_alt = dive_start_coords[2]
        
        print(f"🎯 İlk waypoint (Geopy):")
        print(f"   Enlem: {self.target_lat:.6f}")
        print(f"   Boylam: {self.target_lon:.6f}")
        print(f"   Yükseklik: {self.target_alt}m")
        
        # İlk waypoint gönder (Geopy waypoint)
        if not self.send_waypoint():
            print("❌ Geopy waypoint gönderilemedi")
            return False
        
        # AUTO moduna geç
        if not self.set_mode('AUTO'):
            print("❌ AUTO moda geçilemedi")
            return False
        
        geopy_waypoint_reached = False
        fbwa_mode_set = False
        dive_completed = False
        retry_count = 0
        max_retries = 3
        
        start_time = time.time()
        timeout = 300  # 5 dakika timeout
        
        print("🎯 Geopy waypoint'e yaklaşma başlatıldı...")
        
        while (time.time() - start_time) < timeout and not dive_completed and retry_count < max_retries:
            try:
                # AŞAMA 1: Geopy waypoint'e ulaşma
                if not geopy_waypoint_reached:
                    # Geopy waypoint'e mesafe
                    geopy_distance = self.calculate_distance(
                        self.current_lat, self.current_lon,
                        self.target_lat, self.target_lon
                    )
                    
                    print(f"🎯 Geopy waypoint'e mesafe: {geopy_distance:.2f}m")
                    
                    # Geopy waypoint'e ulaştıysa
                    if geopy_distance <= 5:  # 15m tolerance
                        print("✅ Geopy waypoint'e ulaşıldı!")
                        geopy_waypoint_reached = True
                        
                        # QR hedefini geri yükle (dalış kontrolü için)
                        self.target_lat = original_qr_lat
                        self.target_lon = original_qr_lon
                        self.target_alt = original_qr_alt
                        
                        print("🎮 FBWA moduna geçiliyor...")
                        if self.set_mode('FBWA'):
                            fbwa_mode_set = True
                            print("✅ FBWA modu aktif!")
                            
                            # QR taramayı aktif et
                            global qr_scan_active
                            with qr_scan_lock:
                                qr_scan_active = True
                                print("🔍 QR tarama aktif edildi")
                        else:
                            print("❌ FBWA moduna geçilemedi")
                            break
                
                        # AŞAMA 2: FBWA modunda dalış kontrolü
                        if fbwa_mode_set and geopy_waypoint_reached:                            # 40x40 koşulunu kontrol et
                            dive_status = self.check_dive_conditions()
                            
                            # 45° dalış başlatma koşulu
                            if dive_status['dive_ready'] and fbwa_mode_set and not dive_completed:
                                print("🎯 45° dalış koşulları sağlandı!")
                                
                                if self.qr_dive_maneuver():
                                    print("🎉 45° QR dalış manevrası başarıyla tamamlandı!")
                                    dive_completed = True
                                    break
                                else:
                                    print("❌ QR dalış manevrası başarısız!")
                                    retry_count += 1
                                    if retry_count < max_retries:
                                        print(f"🔄 Yeniden deneme ({retry_count}/{max_retries})")
                                        # Tekrar Geopy waypoint'e dön
                                        geopy_waypoint_reached = False
                                        fbwa_mode_set = False
                                        
                                        # Geopy waypoint'i tekrar hedef yap
                                        self.target_lat = dive_start_coords[0]
                                        self.target_lon = dive_start_coords[1]
                                        self.target_alt = dive_start_coords[2]
                                        
                                        if not self.send_waypoint():
                                            print("❌ Geopy waypoint yeniden gönderilemedi")
                                            break
                                        
                                        if not self.set_mode('AUTO'):
                                            print("❌ AUTO moda geçilemedi")
                                            break
                                    else:
                                        print("❌ Maksimum deneme sayısına ulaşıldı!")
                                        break
                
                print(f"📍 Konum: {self.current_lat:.6f}, {self.current_lon:.6f}")
                print(f"📏 AGL: {self.current_agl:.2f}m, Hız: {self.current_airspeed:.2f}m/s")
                
                time.sleep(1)
                
            except KeyboardInterrupt:
                print("❌ Kullanıcı tarafından iptal edildi")
                break
            except Exception as e:
                print(f"❌ Navigasyon hatası: {e}")
                break
        
        # Sonuç değerlendirmesi
        if dive_completed:
            print("🎉 QR dalış görevi başarıyla tamamlandı!")
            return True
        else:
            print("❌ Görev tamamlanamadı!")
            return False

    def retry_approach_maneuver(self):
            """Uzaklaşma ve yeniden yaklaşma manevrası"""
            print("🔄 Uzaklaşma ve yeniden yaklaşma manevrası başlatılıyor...")
            
            # FBWA moduna geç (manuel kontrol için)
            if not self.set_mode('FBWA'):
                print("❌ FBWA moduna geçilemedi, manöver iptal edildi.")
                return False
            
            # 1. Aşama: Uzaklaşma manevrası
            print("🚀 Uzaklaşma manevrası...")
            
            # Hedefin tersi yönünde uzaklaş
            escape_start_time = time.time()
            escape_duration = 15  # 15 saniye uzaklaş
            
            while (time.time() - escape_start_time) < escape_duration:
                # Throttle artır, elevator yukarı (tırmanış)
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    1500, 1700, 1400, 1500, 0, 0, 0, 0  # Tırmanış ve uzaklaşma
                )
                
                current_distance = self.calculate_distance(
                    self.current_lat, self.current_lon,
                    self.target_lat, self.target_lon
                )
                
                print(f"🔄 Uzaklaşma - Mesafe: {current_distance:.2f}m, AGL: {self.current_agl:.2f}m, Hız: {self.current_airspeed:.2f}m/s")
                
                # 20 metre irtifaya ulaştığında uzaklaşmaya devam et
                if self.current_agl >= 20.0:
                    print(f"✅ Hedef irtifaya ulaşıldı: {self.current_agl:.2f}m")
                    break
                
                time.sleep(0.1)
            
            # 2. Aşama: 20 metre irtifada stabilize ol
            print("📏 20 metre irtifada stabilizasyon...")
            stabilize_start_time = time.time()
            
            while (time.time() - stabilize_start_time) < 8:  # 10 saniye stabilize
                # 20 metre irtifayı koru
                if self.current_agl < 3.0:
                    # Çok alçak, tırman
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        1500, 1600, 1400, 1500, 0, 0, 0, 0
                    )
                elif self.current_agl > 15.0:
                    # Çok yüksek, alçal
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        1500, 1200, 1300, 1500, 0, 0, 0, 0
                    )
                else:
                    # Stabilize
                    self.master.mav.rc_channels_override_send(
                        self.master.target_system,
                        self.master.target_component,
                        1500, 1500, 1500, 1500, 0, 0, 0, 0
                    )
                
                print(f"⚖️ Stabilizasyon - AGL: {self.current_agl:.2f}m, Hız: {self.current_airspeed:.2f}m/s")
                time.sleep(0.1)
            
            # RC override temizle
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            print("🔁 RC override temizlendi.")
            
            # 3. Aşama: Hedef irtifayı güncelle ve yeni waypoint gönder
            print("📍 Yeni waypoint (20m irtifada) gönderiliyor...")
            
            # Hedef irtifayı güncelle
            original_alt = self.target_alt
            self.target_alt = 10.0
            
            # Yeni waypoint gönder
            if not self.send_waypoint():
                print("❌ Yeni waypoint gönderilemedi")
                self.target_alt = original_alt  # Orijinal irtifayı geri yükle
                return False
            
            # AUTO moduna geç
            if not self.set_mode('AUTO'):
                print("❌ AUTO moda geçilemedi")
                self.target_alt = original_alt  # Orijinal irtifayı geri yükle
                return False
            
            print("🚀 20 metre irtifada hedefe yeniden yaklaşma başlatıldı...")
            
            # 4. Aşama: Hedefe yaklaşma (20m irtifada)
            approach_start_time = time.time()
            approach_timeout = 120  # 2 dakika timeout
            
            while (time.time() - approach_start_time) < approach_timeout:
                current_distance = self.calculate_distance(
                    self.current_lat, self.current_lon,
                    self.target_lat, self.target_lon
                )
                
                print(f"🎯 Yeniden yaklaşma - Mesafe: {current_distance:.2f}m, AGL: {self.current_agl:.2f}m")
                
                # Hedefe yaklaştığında (kamera mesafesine) çık
                if current_distance <= (self.camera_approach_distance + 5):  # 5m buffer
                    print(f"✅ Yeniden yaklaşma tamamlandı! Mesafe: {current_distance:.2f}m")
                    break
                
                time.sleep(1)
            
            # Orijinal hedef irtifayı geri yükle
            self.target_alt = original_alt
            
            print("🔄 Uzaklaşma ve yeniden yaklaşma manevrası tamamlandı!")
            return True

def main():
    global running
    """Ana fonksiyon"""
    print("🛩️ Talon Kamera Navigasyon Sistemi")
    print("=" * 50)
    
    navigator = TalonCameraNavigator()
    
    # Hedef bilgilerini göster
    print(f"📍 Hedef Koordinatlar:")
    print(f"   Enlem: {navigator.target_lat}")
    print(f"   Boylam: {navigator.target_lon}")
    print(f"   Yükseklik: {navigator.target_alt}m")
    print(f"📹 Kamera yaklaşma mesafesi: {navigator.camera_approach_distance}m")
    print("=" * 50)
    
    # Navigasyonu başlat
    try:
        # Navigasyonu başlat
        success = navigator.navigate_to_waypoint()
    finally:
        running = False  # <-- Thread'in sonlanması için
    
    if success:
        print("🎉 Görev başarıyla tamamlandı!")
    else:
        print("❌ Görev başarısız!")

if __name__ == '__main__':
    main()