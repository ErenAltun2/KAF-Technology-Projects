import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading
from pymavlink import mavutil
import logging
import math
from enum import Enum, auto
import numpy as np
from detect_tracking5 import detect_target_and_get_output

class MissionParameters:
    """Görev parametrelerini merkezi olarak yöneten sınıf - SADELEŞTIRILDI"""
    def __init__(self):
        # Genel parametreler
        self.patrol_altitude = 5.0  # Devriye irtifası (metre)
        self.hover_altitude = 1.0   # Bekleme irtifası (metre) - SABIT
        self.patrol_speed = 2.0     # Devriye hızı (m/s)

        # Hedef takip parametreleri - SADELEŞTIRILDI
        self.target_lost_timeout = 5.0      # Hedef kaybı timeout (saniye)
        self.hover_duration = 10.0          # Hedef üzerinde bekleme süresi (saniye)
        self.move_interval = 0.2            # Hareket güncelleme aralığı (saniye)
        self.min_detection_frames = 3       # minimum tespit frame sayısı

        
        # PID parametreleri - AYNI
        self.pid_params = {
            'x': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000},
            'y': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000}
        }
        
        # Görüntü işleme parametreleri - SADELEŞTIRILDI
        self.image_center = (320, 240)  # Görüntü merkezi (piksel)
        self.target_tolerance = 100      # SABIT tolerans değeri
        
        # Görev durumları - AYNI
        self.require_hexagon = True     # Altıgen hedefi gerekli mi?
        self.hexagon_completed = False  # Altıgen tamamlandı mı?
        self.triangle_completed = False

class PID:
    def __init__(self, kp, ki, kd, imax=1000):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.imax = imax
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_t = None

    def update(self, err):
        now = time.time()
        if self.prev_t is None:
            self.prev_t = now
            self.prev_err = err
            return 0.0
        
        dt = now - self.prev_t
        de = err - self.prev_err
        self.integral += err * dt
        self.integral = max(min(self.integral, self.imax), -self.imax)

        out = (self.kp * err) + (self.ki * self.integral) + (self.kd * de / dt)
        self.prev_err, self.prev_t = err, now
        return out

class DroneState(Enum):
    TAKEOFF = auto()
    PATROL = auto()
    TARGET_TRACK = auto()
    DESCENDING = auto()  # Bu satırı ekle
    HOVERING = auto()
    RETURNING = auto()
    SEARCHING_HEXAGON = auto()
    SEARCHING_TRIANGLE = auto()
    
class PatrolDrone(Node):
    def __init__(self, patrol_coordinates):
        super().__init__('patrol_drone')
        
        # Parametre yönetimi
        self.params = MissionParameters()
        
        # ROS2 görüntü aboneliği
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.bridge = CvBridge()
        
        # DroneKit bağlantısı
        self.vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
        self.get_logger().info("DroneKit bağlantısı kuruldu.")
        self.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Yönlendirme davranışını kapat yaw kontrolü 0

            # YENI: Devriye koordinatları ve arama durumları
        self.patrol_coordinates = patrol_coordinates
        
        # YENI: Her waypoint için arama durumu tanımla
        self.waypoint_search_enabled = [
            True,   # WP1: (-35.36278558, 149.16505271) - ARAMA YAP
            True,   # WP2: (-35.36275136, 149.16527146) - ARAMA YAP  
            False,   # WP3: (-35.36288332, 149.16539732) - ARAMA YAPMA
            True,  # WP4: (-35.36309593, 149.16534038) - ARAMA YAP
            True    # WP5: (-35.36300000, 149.16510000) - ARAMA YAP
    ]
        # Devriye koordinatları
        self.patrol_coordinates = patrol_coordinates
        self.current_waypoint_index = 0

        # Tolerans takibi için
        self.tolerance_update_interval = 0.5  # Tolerans kontrol aralığı (saniye)
        self.last_tolerance_check = 0
        
        # Drone durumu
        self.state = DroneState.TAKEOFF
        self.current_target_type = None
        
        self.altitude_descent_started = False  # İrtifa düşürme başladı mı?



        # Hedef takip geçmişi
        self.target_history = []
        self.max_history = 8
        self.target_lost_time = 0
        self.last_target_time = 0
        self.last_move_time = 0
        self.hover_start_time = 0
        
        # YENİ: Kararlı tespit değişkenleri
        self.detection_counter = 0
        self.stable_target_detected = False
        self.last_stable_detection_time = 0
        
        # PID kontrolcüleri
        self.pid_x = PID(**self.params.pid_params['x'])
        self.pid_y = PID(**self.params.pid_params['y'])
        
        # Görüntüleme penceresi
        self.window_name = "Uluslararası İHA Kaf Takımı"
        cv2.namedWindow(self.window_name)
        
        # Ana kontrol thread'i başlat
        self.control_thread = threading.Thread(target=self.main_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()

        self.search_for = None  # Aranacak hedef türü
        self.hiz_limit = 0.3  # Hız limitleri (m/s)
        self.patrol_coordinates = patrol_coordinates
        self.original_patrol_coordinates = patrol_coordinates.copy()  # YENİ: Orijinal koordinatları sakla
        self.current_waypoint_index = 0


    def reset_detection_state(self):
        """Tespit durumunu sıfırla"""
        self.detection_counter = 0
        self.stable_target_detected = False
        self.current_target_type = None
        self.target_lost_time = 0
        self.altitude_descent_started = False
        
        # YENİ: Ortalama zamanlayıcısını da sıfırla
        if hasattr(self, 'centered_time'):
            delattr(self, 'centered_time')

    def descending_behavior(self):
        """İrtifa düşürme davranışı - Hedef tespiti kapalı"""
        current_alt = self.vehicle.location.global_relative_frame.alt
        target_alt = 1.0
        
        # İrtifa düşürme komutu ver
        location = LocationGlobalRelative(
            self.vehicle.location.global_relative_frame.lat,
            self.vehicle.location.global_relative_frame.lon,
            target_alt
        )
        self.vehicle.simple_goto(location, groundspeed=0.5)
        
        # İrtifa kontrolü
        if current_alt <= 1.3:  # Hedefe yaklaştı
            self.get_logger().info(f"✅ İrtifa hedefine ulaşıldı ({current_alt:.1f}m) - HOVERING başlıyor!")
            self.state = DroneState.HOVERING
            self.hover_start_time = time.time()
            self.search_for = "disabled"
        else:
            # Henüz hedefe ulaşmadı - durumu raporla
            if hasattr(self, '_last_alt_report'):
                if abs(current_alt - self._last_alt_report) > 0.5:  # 0.5m değişim olduğunda rapor et
                    self.get_logger().info(f"⬇️ İrtifa düşürülüyor: {current_alt:.1f}m → {target_alt}m")
                    self._last_alt_report = current_alt
            else:
                self._last_alt_report = current_alt

    def main_control_loop(self):
        """Ana kontrol döngüsü"""
        try:
            self.arm_and_takeoff(self.params.patrol_altitude)
            self.state = DroneState.PATROL
            
            while True:
                if self.state is None:
                    time.sleep(1)
                    continue
                
                current_state = self.state
                
                if current_state == DroneState.TAKEOFF:
                    pass
                elif current_state == DroneState.PATROL:
                    self.patrol_behavior()
                elif current_state == DroneState.TARGET_TRACK:
                    self.target_tracking_behavior()
                elif current_state == DroneState.DESCENDING:  # Bu bloğu ekle
                    self.descending_behavior()
                elif current_state == DroneState.HOVERING:
                    self.hovering_behavior()
                elif current_state == DroneState.RETURNING:
                    self.returning_behavior()
                elif current_state == DroneState.SEARCHING_HEXAGON:
                    self.search_behavior(target_type='hexagon')
                elif current_state == DroneState.SEARCHING_TRIANGLE:
                    self.search_behavior(target_type='triangle')

                time.sleep(0.1)
                
        except Exception as e:
            self.get_logger().error(f"Kontrol döngüsünde hata: {e}")
            self.emergency_land()

    def patrol_behavior(self):
        """Devriye davranışı - waypoint'ler arasında dolaş"""
        current_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        current_location = self.vehicle.location.global_relative_frame
        
        # Mevcut waypoint'e olan mesafe
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon,
            current_waypoint[0], current_waypoint[1]
        )
        
        # Waypoint'e yaklaştıysa bir sonrakine geç
        if distance < 2.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
            next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
            
            # YENI: Waypoint'in arama durumunu logla
            search_status = "ARAMA" if self.waypoint_search_enabled[self.current_waypoint_index] else "GEÇİŞ"
            self.get_logger().info(f"📍 WP{self.current_waypoint_index+1}: {next_waypoint} ({search_status})")
            
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        # Kararlı hedef algılandıysa takibe geç
        if self.stable_target_detected:
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

    def target_tracking_behavior(self):
        """Hedef takip davranışı - Düzeltilmiş"""
        current_time = time.time()
        
        # Kararlı hedef kaybedildiyse geri sayım başlat
        if not self.stable_target_detected:
            if self.target_lost_time == 0:
                self.target_lost_time = current_time
                self.get_logger().info("⚠️ Hedef kaybedildi...")
            
            # Timeout süresi dolunca
            elif current_time - self.target_lost_time > self.params.target_lost_timeout:
                current_alt = self.vehicle.location.global_relative_frame.alt
                if current_alt <= 2.0:  # Düşük irtifadaysa hovering'e geç
                    self.get_logger().info("🚁 Düşük irtifada - Hovering başlatılıyor...")
                    self.state = DroneState.HOVERING
                    self.hover_start_time = time.time()
                    self.search_for = "disabled"
                else:
                    self.get_logger().info("🔄 Hedef kayıp - Devriyeye dönülüyor...")
                    self.state = DroneState.RETURNING
                
                self.target_lost_time = 0
                self.detection_counter = 0
                self.target_history.clear()
        else:
            # Hedef görülüyor - kayıp zamanını sıfırla
            self.target_lost_time = 0
            self.last_target_time = current_time
            
            # Hedef takip geçmişi var mı kontrol et
            if len(self.target_history) >= 2:
                avg_dx = sum([h[0] for h in self.target_history[-2:]]) / 2
                avg_dy = sum([h[1] for h in self.target_history[-2:]]) / 2
                avg_distance = math.sqrt(avg_dx**2 + avg_dy**2)
                
                current_alt = self.vehicle.location.global_relative_frame.alt
                
                # HEDEF ORTALANDI
                if avg_distance < self.params.target_tolerance:
                    # SADECE İLK KEZ ORTALANDIĞINDA ve YÜKSEK İRTİFADAYSA İNDİR
                    if not self.altitude_descent_started and current_alt > 1.5:
                        self.get_logger().info(f"🎯 Hedef ortalandı! İrtifa DESCENDING durumuna geçiliyor...")
                        self.state = DroneState.DESCENDING  # DESCENDING durumuna geç
                        self.altitude_descent_started = True
                        
                # HEDEF HENÜZ ORTALANMADI - TAKİP ET
                else:
                    if current_time - self.last_move_time > self.params.move_interval:
                        err_x, err_y = self.target_history[-1]
                        
                        vy_body = self.pid_y.update(err_y)
                        vx_body = self.pid_x.update(err_x)
                        
                        vy = max(min(vy_body, self.hiz_limit), -self.hiz_limit)
                        vx = max(min(vx_body, self.hiz_limit), -self.hiz_limit)
                        
                        self.send_body_velocity(vx, vy, 0, 0)
                        print(f"{self.current_target_type} Mesafe:{avg_distance:.1f}px vx={vx:.2f}, vy={vy:.2f}")
                        self.last_move_time = current_time
   
    def hovering_behavior(self):
        """Hovering davranışı - 1 metrede 10 saniye bekleme"""
        current_time = time.time()
        elapsed_time = current_time - self.hover_start_time
        remaining_time = self.params.hover_duration - elapsed_time
        
        if not hasattr(self, '_hovering_search_disabled'):
            self.search_for = "disabled"
            self._hovering_search_disabled = True
        # İrtifayı kontrol et - sadece büyük sapmalarda düzelt
        current_location = self.vehicle.location.global_relative_frame
        altitude_diff = abs(current_location.alt - 1.0)
        
        if altitude_diff > 0.3:
            location = LocationGlobalRelative(
                current_location.lat,
                current_location.lon,
                1.0
            )
            self.vehicle.simple_goto(location, groundspeed=0.2)
        else:
            # Hareketsiz dur
            self.send_body_velocity(0, 0, 0, 0)
        
        # Bekleme süresi kontrolü - her saniye log
        if remaining_time > 0:
            if int(remaining_time) != getattr(self, '_last_logged_time', -1):
                self.get_logger().info(f"⏳ {self.current_target_type} üzerinde bekleniyor... {int(remaining_time)}s kaldı")
                self._last_logged_time = int(remaining_time)
        else:
            # 10 saniye tamamlandı - görev durumuna göre işlem yap
            if self.current_target_type == 'hexagon':
                # ALTIGEN GÖREVİ TAMAMLANDI
                self.params.hexagon_completed = True
                self.get_logger().info("🎉 Altıgen görevi tamamlandı!")
                
                # ÜÇGEN HENÜZ TAMAMLANMADI - ÜÇGEN ARAMAYA GEÇ
                if not self.params.triangle_completed:
                    self.get_logger().info("🔍 Üçgen arama moduna geçiliyor...")
                    self.state = DroneState.RETURNING  # RETURNING önce yükseklik ayarı yapar
                    self.search_for = None  # Tespit tekrar açılsın
                else:
                    # İKİ GÖREV DE TAMAM - RTL
                    self.get_logger().info("🛬 Tüm görevler tamamlandı - RTL moduna geçiliyor...")
                    self.vehicle.mode = VehicleMode("RTL")
                    self.state = None
                    
            elif self.current_target_type == 'triangle':
                # ÜÇGEN GÖREVİ TAMAMLANDI
                self.params.triangle_completed = True
                self.get_logger().info("🎉 Üçgen görevi tamamlandı!")
                self.get_logger().info("🛬 RTL moduna geçiliyor...")
                self.vehicle.mode = VehicleMode("RTL")
                self.state = None

            
    def returning_behavior(self):
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        # Tespit durumunu sıfırla
        self.reset_detection_state()
        
        # Önce devriye irtifasına yüksel
        if current_alt < self.params.patrol_altitude - 1.0:
            self.get_logger().info(f"⬆️ Yükseliyor... {current_alt:.1f}m → {self.params.patrol_altitude}m")
            location = LocationGlobalRelative(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.params.patrol_altitude
            )
            self.vehicle.simple_goto(location, groundspeed=self.hiz_limit)
            time.sleep(1.0)
            return
        
        # YENİ: Altıgen tamamlandı ama üçgen henüz tamamlanmadıysa devriyeye devam et
        if self.params.hexagon_completed and not self.params.triangle_completed:
            self.get_logger().info("🔍 Üçgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_TRIANGLE
            self.find_nearest_waypoint_and_go()
            self.target_history.clear()
            self.target_lost_time = 0
            return
        
        # Diğer durumlar...
        elif not self.params.hexagon_completed:
            self.get_logger().info("🔍 Altıgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_HEXAGON
            self.find_nearest_waypoint_and_go()
        else:
            # Normal patrol devam et
            self.state = DroneState.PATROL
            self.find_nearest_waypoint_and_go()

        # Geçmişi temizle
        self.target_history.clear()
        self.target_lost_time = 0

    def search_behavior(self, target_type):
        """Belirli bir hedef türü için arama davranışı - Waypoint bazlı"""
        current_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        current_location = self.vehicle.location.global_relative_frame
        
        # Mevcut waypoint'e olan mesafe
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon,
            current_waypoint[0], current_waypoint[1]
        )
        
        # Waypoint'e yaklaştıysa bir sonrakine geç
        if distance < 2.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
            next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
            
            # YENI: Waypoint'in arama durumunu logla
            search_status = "ARAMA" if self.waypoint_search_enabled[self.current_waypoint_index] else "GEÇİŞ"
            self.get_logger().info(f"🔍 {target_type} aranıyor - WP{self.current_waypoint_index+1}: {next_waypoint} ({search_status})")
            
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        # YENI: Sadece arama yapılmasına izin verilen waypoint'lerde hedef arama
        current_waypoint_allows_search = self.waypoint_search_enabled[self.current_waypoint_index]
        
        if (self.stable_target_detected and 
            self.current_target_type == target_type and 
            self.search_for == target_type and
            current_waypoint_allows_search):  # YENI KOŞUL
            self.get_logger().info(f"🎯 {target_type} kararlı şekilde tespit edildi!")
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

    def image_callback(self, msg):
        """Görüntü işleme callback'i - Waypoint bazlı arama kontrolü ile"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # HOVERING ve DESCENDING SIRASINDA GÖRÜNTÜ İŞLEME DURDUR
        if self.state in [DroneState.HOVERING, DroneState.DESCENDING]:
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
            return
        
        # YENI: Waypoint bazlı arama kontrolü
        current_waypoint_allows_search = self.waypoint_search_enabled[self.current_waypoint_index]
        
        if self.state == DroneState.SEARCHING_HEXAGON:
            self.search_for = "hexagon" if current_waypoint_allows_search else "disabled"
        elif self.state == DroneState.SEARCHING_TRIANGLE:
            self.search_for = "triangle" if current_waypoint_allows_search else "disabled"
        elif self.state == DroneState.PATROL:
            if current_waypoint_allows_search:  # Bu waypoint'te arama yapılacak
                if not self.params.hexagon_completed:
                    self.search_for = "hexagon"
                elif not self.params.triangle_completed:
                    self.search_for = "triangle"
                else:
                    self.search_for = "disabled"
            else:  # Bu waypoint'te arama yapılmayacak
                self.search_for = "disabled"
        elif self.state == DroneState.RETURNING:
            self.search_for = "disabled"
        elif self.state in [DroneState.TARGET_TRACK]:
            pass  # Mevcut search_for değerini koru
        else:
            self.search_for = "disabled"

        # Geri kalan görüntü işleme kodu aynı kalacak...
        vis, t_type, center = detect_target_and_get_output(cv_image, model=None, search_for=self.search_for)

        # Hedef tespit mantığı
        if t_type in ['hexagon', 'triangle']:
            self.detection_counter += 1
            self.current_target_type = t_type
            
            if self.detection_counter >= self.params.min_detection_frames:
                self.stable_target_detected = True
                self.last_stable_detection_time = time.time()
        else:
            self.detection_counter = max(0, self.detection_counter - 2)
            if self.detection_counter == 0:
                self.stable_target_detected = False
                self.current_target_type = None

        # Kararlı hedef merkezi bulunduysa takip mantığını uygula
        if self.stable_target_detected and center:
            err_x = center[0] - self.params.image_center[0]
            err_y = center[1] - self.params.image_center[1]
            
            self.target_history.append((err_x, err_y))
            
            if self.state == DroneState.TARGET_TRACK:
                current_time = time.time()
                if current_time - self.last_move_time > self.params.move_interval:
                    
                    # GELİŞTİRİLMİŞ ORTALAMA KONTROLÜ
                    distance_from_center = math.sqrt(err_x**2 + err_y**2)
                    current_tolerance = self.params.target_tolerance
                    
                    if distance_from_center < current_tolerance:
                        current_alt = self.vehicle.location.global_relative_frame.alt
                        
                        # Zamana dayalı ortalama kontrolü
                        if not hasattr(self, 'centered_time'):
                            self.centered_time = current_time
                            self.get_logger().info(f"🎯 Hedef ortalanıyor... (mesafe: {distance_from_center:.1f}px)")
                        
                        # 1.5 saniye boyunca ortalı kaldıysa descend et
                        elif current_time - self.centered_time > 1.5:
                            if not self.altitude_descent_started and current_alt > 2.0:
                                self.get_logger().info(f"🎯 Hedef {current_time - self.centered_time:.1f}s ortalı kaldı! DESCENDING...")
                                self.state = DroneState.DESCENDING
                                self.altitude_descent_started = True
                                # Zamanlayıcıyı temizle
                                if hasattr(self, 'centered_time'):
                                    delattr(self, 'centered_time')
                        else:
                            # Hala ortalanıyor, bekleme süresini göster
                            remaining_time = 1.5 - (current_time - self.centered_time)
                            print(f"🎯 Ortalanıyor... {remaining_time:.1f}s kaldı (mesafe: {distance_from_center:.1f}px)")
                            
                    else:
                        # Hedef ortalı değilse zamanlayıcıyı sıfırla
                        if hasattr(self, 'centered_time'):
                            delattr(self, 'centered_time')
                            self.get_logger().info("⚠️ Hedef ortalama dışı çıktı, yeniden ortalanıyor...")
                        
                        # PID ile hareket ettir
                        vy_body = self.pid_y.update(err_y)
                        vx_body = self.pid_x.update(err_x)
                        
                        vy = max(min(vy_body, self.hiz_limit), -self.hiz_limit)
                        vx = max(min(vx_body, self.hiz_limit), -self.hiz_limit)
                        
                        self.send_body_velocity(vx, vy, 0, 0)
                        print(f"{self.current_target_type} Tol:{current_tolerance}px vx={vx:.2f}, vy={vy:.2f}, mesafe:{distance_from_center:.1f}px")
                    
                    self.last_move_time = current_time
            
        cv2.imshow(self.window_name, vis)
        cv2.waitKey(1)

    def send_body_velocity(self, vx, vy, vz, yaw_rate):
        """Drone'a hız komutu gönder"""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def arm_and_takeoff(self, target_altitude):
        """Drone'u arm et ve kaldır"""
        self.get_logger().info("🚁 Başlatılıyor...")
        while not self.vehicle.is_armable:
            time.sleep(1)
            
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            time.sleep(1)
        
        self.get_logger().info("🛫 Kalkış...")
        self.vehicle.simple_takeoff(target_altitude)
        
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                self.get_logger().info(f"✅ Yükseklik: {alt:.1f}m")
                break
            time.sleep(1)
        
        # İlk waypoint'e git
        first_waypoint = self.patrol_coordinates[0]
        self.goto_waypoint(first_waypoint[0], first_waypoint[1])

    def goto_waypoint(self, lat, lon, altitude=None):
        """Belirtilen waypoint'e git"""
        if altitude is None:
            altitude = self.params.patrol_altitude
        location = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.simple_goto(location, groundspeed=self.params.patrol_speed)
        
    def find_nearest_waypoint_and_go(self):
        """Bir sonraki sıralı waypoint'e git (sıralı geçiş)"""
        # Mevcut waypoint index'i koruyarak bir sonraki waypoint'e geç
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
        
        next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        self.get_logger().info(f"📍 Sıradaki waypoint: WP{self.current_waypoint_index+1}: {next_waypoint}")
    
    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        """İki GPS koordinatı arasındaki mesafeyi hesapla"""
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)

    def emergency_land(self):
        """Acil iniş prosedürü"""
        self.get_logger().error("🆘 Acil iniş yapılıyor!")
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        self.vehicle.close()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    
    # Waypoint koordinatları
    patrol_coordinates = [
        (-35.36278558, 149.16505271),  # WP1 - ARAMA YAP
        (-35.36275136, 149.16527146),  # WP2 - ARAMA YAP  
        (-35.36288332, 149.16539732),  # WP3 - ARAMA YAP
        (-35.36309593, 149.16534038),  # WP4 - SADECE GEÇ
        (-35.36300000, 149.16510000)   # WP5 - ARAMA YAP
    ]
    
    patrol_drone = PatrolDrone(patrol_coordinates)
    
    try:
        rclpy.spin(patrol_drone)
    except KeyboardInterrupt:
        patrol_drone.get_logger().info("🛑 Sistem kapatılıyor...")
    finally:
        patrol_drone.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()