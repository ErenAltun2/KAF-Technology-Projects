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
import kutu_acma
import stream_server

class MissionParameters:
    """Görev parametrelerini merkezi olarak yöneten sınıf"""
    def __init__(self):
        # Genel parametreler
        self.patrol_altitude = 7.0  # Devriye irtifası (metre)
        self.hover_altitude = 1.0   # Bekleme irtifası (metre)
        self.patrol_speed = 3.5     # Devriye hızı (m/s)
        
        # Tur sayacı parametreleri
        self.max_patrol_tours = 3       # Maksimum devriye tur sayısı
        self.current_tour_count = 0     # Mevcut tur sayacı
        
        # Hedef takip parametreleri
        self.target_lost_timeout = 5.0      # Hedef kaybı timeout (saniye)
        self.hover_duration = 5.0          # Hedef üzerinde bekleme süresi (saniye)
        self.move_interval = 0.2            # Hareket güncelleme aralığı (saniye)
        self.min_detection_frames = 1       # minimum tespit frame sayısı
        
        # PID parametreleri
        self.pid_params = {
            'x': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000},
            'y': {'kp': 0.5, 'ki': 0.03, 'kd': 0.02, 'imax': 1000}
        }
        
        # Görüntü işleme parametreleri
        self.image_center = (320, 240)  # Görüntü merkezi (piksel)
        self.target_tolerance = 350    # Tolerans değeri
        
        # Görev durumları
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
    DESCENDING = auto()
    HOVERING = auto()
    RETURNING = auto()
    SEARCHING_HEXAGON = auto()
    SEARCHING_TRIANGLE = auto()
    
class PatrolDrone:
    def __init__(self, patrol_coordinates):
        # Parametre yönetimi
        self.params = MissionParameters()
        
        # Logging setup
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # USB Kamera başlatma
        self.cap = cv2.VideoCapture(0)  # USB kamera (/dev/video0)
        if not self.cap.isOpened():
            self.logger.error("USB kamera açılamadı!")
            return
        stream_server.start_server(host="0.0.0.0", port=5001)

        # Kamera çözünürlük ayarları
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.vehicle = connect('/dev/serial0',baud=57600, wait_ready=True)
        print("DroneKit bağlantısı kuruldu.")
        
        self.vehicle.parameters['WP_YAW_BEHAVIOR'] = 0 # Yönlendirme davranışını kapat
        
        # Devriye koordinatları
        self.patrol_coordinates = patrol_coordinates
        self.current_waypoint_index = 0
        self.original_patrol_coordinates = patrol_coordinates.copy()
     #********************************************************************************************   
        # YENI EKLEME: Her waypoint için arama durumu tanımla
        self.waypoint_search_enabled =[
    False,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    False,
    False,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True,
    True
]
        #********************************************************************
        # Tolerans takibi için
        self.tolerance_update_interval = 0.5  # Tolerans kontrol aralığı (saniye)
        self.last_tolerance_check = 0
        
        # Drone durumu
        self.state = DroneState.TAKEOFF
        self.current_target_type = None
        self.altitude_descent_started = False

        # Hedef takip geçmişi
        self.target_history = []
        self.max_history = 8
        self.target_lost_time = 0
        self.last_target_time = 0
        self.last_move_time = 0
        self.hover_start_time = 0
        
        # Kararlı tespit değişkenleri
        self.detection_counter = 0
        self.stable_target_detected = False
        self.last_stable_detection_time = 0
        
        # PID kontrolcüleri
        self.pid_x = PID(**self.params.pid_params['x'])
        self.pid_y = PID(**self.params.pid_params['y'])
        
   
        
        self.search_for = None  # Aranacak hedef türü
        self.hiz_limit = 0.15  # Hız limitleri (m/s)
        
        # Running flag
        self.running = True
        
        # Ana kontrol thread'i başlat
        self.control_thread = threading.Thread(target=self.main_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Kamera thread'i başlat
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()

    def camera_loop(self):
        """USB kamera görüntü işleme döngüsü"""
        frame_i = 0
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                self.logger.warning("Kamera görüntüsü alınamadı!")
                continue
                
            self.process_frame(frame)
            frame_i += 1
            if frame_i % 3 == 0:
                stream_server.update_frame(frame)
            time.sleep(0.03)  # ~30 FPS

    def process_frame(self, cv_image):
        """Görüntü işleme fonksiyonu"""
        # HOVERING ve DESCENDING SIRASINDA GÖRÜNTÜ İŞLEME DURDUR
        if self.state in [DroneState.HOVERING, DroneState.DESCENDING]:
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

        # Geri kalan kod aynı kalacak...
        vis, t_type, center = detect_target_and_get_output(cv_image, model=None, search_for=self.search_for)
        stream_server.update_frame(vis)

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
                    current_tolerance = self.params.target_tolerance
                    
                    if abs(err_x) < current_tolerance and abs(err_y) < current_tolerance:
                        current_alt = self.vehicle.location.global_relative_frame.alt
                        
                        if not self.altitude_descent_started and current_alt > 1.0:
                            self.logger.info(f"🎯 Hedef ortalandı! DESCENDING durumuna geçiliyor...")
                            self.state = DroneState.DESCENDING
                            self.altitude_descent_started = True
                            
                    else:
                        vy_body = self.pid_y.update(err_y)
                        vx_body = self.pid_x.update(err_x)
                        
                        vy = max(min(vy_body, self.hiz_limit), -self.hiz_limit)
                        vx = max(min(vx_body, self.hiz_limit), -self.hiz_limit)
                        
                        self.send_body_velocity(vx, vy, 0, 0)
                        print(f"{self.current_target_type} Tol:{current_tolerance}px vx={vx:.2f}, vy={vy:.2f}")
                    self.last_move_time = current_time
            



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
        """İrtifa düşürme davranışı"""
        current_alt = self.vehicle.location.global_relative_frame.alt
        target_alt = 1.0
        
        location = LocationGlobalRelative(
            self.vehicle.location.global_relative_frame.lat,
            self.vehicle.location.global_relative_frame.lon,
            target_alt
        )
        self.vehicle.simple_goto(location, groundspeed=0.5)
        
        if current_alt <= 1.3:
            self.logger.info(f"✅ İrtifa hedefine ulaşıldı ({current_alt:.1f}m) - HOVERING başlıyor!")
            self.state = DroneState.HOVERING
            self.hover_start_time = time.time()
            self.search_for = "disabled"
        else:
            if hasattr(self, '_last_alt_report'):
                if abs(current_alt - self._last_alt_report) > 0.5:
                    self.logger.info(f"⬇️ İrtifa düşürülüyor: {current_alt:.1f}m → {target_alt}m")
                    self._last_alt_report = current_alt
            else:
                self._last_alt_report = current_alt

    def main_control_loop(self):
        """Ana kontrol döngüsü"""
        try:
            self.arm_and_takeoff(self.params.patrol_altitude)
            self.state = DroneState.PATROL
            
            while self.running:
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
                elif current_state == DroneState.DESCENDING:
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
            self.logger.error(f"Kontrol döngüsünde hata: {e}")
            self.emergency_land()

    def patrol_behavior(self):
        """Devriye davranışı"""
        current_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        current_location = self.vehicle.location.global_relative_frame
        
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon,
            current_waypoint[0], current_waypoint[1]
        )
        
        if distance < 2.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
            next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
            
            # YENI: Waypoint'in arama durumunu logla
            search_status = "ARAMA" if self.waypoint_search_enabled[self.current_waypoint_index] else "GEÇİŞ"
            self.logger.info(f"📍 WP{self.current_waypoint_index+1}: {next_waypoint} ({search_status})")
            
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        if self.stable_target_detected:
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

    def target_tracking_behavior(self):
        """Hedef takip davranışı"""
        current_time = time.time()
        
        if not self.stable_target_detected:
            if self.target_lost_time == 0:
                self.target_lost_time = current_time
                self.logger.info("⚠️ Hedef kaybedildi...")
            
            elif current_time - self.target_lost_time > self.params.target_lost_timeout:
                current_alt = self.vehicle.location.global_relative_frame.alt
                if current_alt <= 2.0:
                    self.logger.info("🚁 Düşük irtifada - Hovering başlatılıyor...")
                    self.state = DroneState.HOVERING
                    self.hover_start_time = time.time()
                    self.search_for = "disabled"
                else:
                    self.logger.info("🔄 Hedef kayıp - Devriyeye dönülüyor...")
                    self.state = DroneState.RETURNING
                
                self.target_lost_time = 0
                self.detection_counter = 0
                self.target_history.clear()
        else:
            self.target_lost_time = 0
            self.last_target_time = current_time
            
            if len(self.target_history) >= 2:
                avg_dx = sum([h[0] for h in self.target_history[-2:]]) / 2
                avg_dy = sum([h[1] for h in self.target_history[-2:]]) / 2
                avg_distance = math.sqrt(avg_dx**2 + avg_dy**2)
                
                current_alt = self.vehicle.location.global_relative_frame.alt
                
                if avg_distance < self.params.target_tolerance:
                    if not self.altitude_descent_started and current_alt > 1.5:
                        self.logger.info(f"🎯 Hedef ortalandı! İrtifa DESCENDING durumuna geçiliyor...")
                        self.state = DroneState.DESCENDING
                        self.altitude_descent_started = True
                        
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
        """Hovering davranışı"""
        current_time = time.time()
        elapsed_time = current_time - self.hover_start_time
        remaining_time = self.params.hover_duration - elapsed_time
        
        if not hasattr(self, '_hovering_search_disabled'):
            self.search_for = "disabled"
            self._hovering_search_disabled = True
            
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
            self.send_body_velocity(0, 0, 0, 0)
        
        if remaining_time > 0:
            if int(remaining_time) != getattr(self, '_last_logged_time', -1):
                self.logger.info(f"⏳ {self.current_target_type} üzerinde bekleniyor... {int(remaining_time)}s kaldı")
                self._last_logged_time = int(remaining_time)
        else:
            if self.current_target_type == 'hexagon':
                self.params.hexagon_completed = True

                #pence acma
                kutu_acma.box_open(self.vehicle,channel=10,degree=180, wait=2.0)
                kutu_acma.box_close(self.vehicle,channel=10,degree=0, wait=2.0)
                time.sleep(3)

                self.logger.info("🎉 Altıgen görevi tamamlandı!")
                
                if not self.params.triangle_completed:
                    self.logger.info("🔍 Üçgen arama moduna geçiliyor...")
                    self.state = DroneState.RETURNING
                    self.search_for = None
                else:
                    self.logger.info("🛬 Tüm görevler tamamlandı - RTL moduna geçiliyor...")
                    self.vehicle.mode = VehicleMode("RTL")
                    self.state = None
                    
            elif self.current_target_type == 'triangle':
                self.params.triangle_completed = True

                #kutu acma
                kutu_acma.box_open(self.vehicle,channel=9, wait=2.0,degree=180)
                kutu_acma.box_close(self.vehicle,channel=9, wait=2.0,degree=0)

                self.logger.info("🎉 Üçgen görevi tamamlandı!")
                self.logger.info("🛬 RTL moduna geçiliyor...")
                self.vehicle.mode = VehicleMode("RTL")
                self.state = None
            
    def returning_behavior(self):
        current_alt = self.vehicle.location.global_relative_frame.alt
        
        self.reset_detection_state()
        
        if current_alt < self.params.patrol_altitude - 1.0:
            self.logger.info(f"⬆️ Yükseliyor... {current_alt:.1f}m → {self.params.patrol_altitude}m")
            location = LocationGlobalRelative(
                self.vehicle.location.global_relative_frame.lat,
                self.vehicle.location.global_relative_frame.lon,
                self.params.patrol_altitude
            )
            self.vehicle.simple_goto(location, groundspeed=self.hiz_limit)
            time.sleep(1.0)
            return
        
        if self.params.hexagon_completed and not self.params.triangle_completed:
            self.logger.info("🔍 Üçgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_TRIANGLE
            self.find_nearest_waypoint_and_go()
            self.target_history.clear()
            self.target_lost_time = 0
            return
        
        elif not self.params.hexagon_completed:
            self.logger.info("🔍 Altıgen arama moduna geçiliyor...")
            self.state = DroneState.SEARCHING_HEXAGON
            self.find_nearest_waypoint_and_go()
        else:
            self.state = DroneState.PATROL
            self.find_nearest_waypoint_and_go()

        self.target_history.clear()
        self.target_lost_time = 0

    def search_behavior(self, target_type):
        """Belirli bir hedef türü için arama davranışı"""
        current_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        current_location = self.vehicle.location.global_relative_frame
        
        distance = self.get_distance_metres(
            current_location.lat, current_location.lon,
            current_waypoint[0], current_waypoint[1]
        )
        
        if distance < 2.0:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
            next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
            
            # YENI: Waypoint'in arama durumunu logla
            search_status = "ARAMA" if self.waypoint_search_enabled[self.current_waypoint_index] else "GEÇİŞ"
            self.logger.info(f"🔍 {target_type} aranıyor - WP{self.current_waypoint_index+1}: {next_waypoint} ({search_status})")
            
            self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        # YENI: Sadece arama yapılmasına izin verilen waypoint'lerde hedef arama
        current_waypoint_allows_search = self.waypoint_search_enabled[self.current_waypoint_index]
        
        if (self.stable_target_detected and 
            self.current_target_type == target_type and 
            self.search_for == target_type and
            current_waypoint_allows_search):  # YENI KOŞUL
            self.logger.info(f"🎯 {target_type} kararlı şekilde tespit edildi!")
            self.state = DroneState.TARGET_TRACK
            self.last_target_time = time.time()
            self.target_history.clear()

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
        self.logger.info("🚁 Başlatılıyor...")
        while not self.vehicle.is_armable:
            self.logger.info("Drone arm edilebilir duruma geçmesi bekleniyor...")
            time.sleep(1)
            
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        
        while not self.vehicle.armed:
            self.logger.info("Arm edilmesi bekleniyor...")
            time.sleep(1)
        
        self.logger.info("🛫 Kalkış...")
        self.vehicle.simple_takeoff(target_altitude)
        
        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            if alt >= target_altitude * 0.95:
                self.logger.info(f"✅ Yükseklik: {alt:.1f}m")
                break
            self.logger.info(f"Yükseliyor: {alt:.1f}m / {target_altitude}m")
            time.sleep(1)
        
        first_waypoint = self.patrol_coordinates[0]
        self.goto_waypoint(first_waypoint[0], first_waypoint[1])

    def goto_waypoint(self, lat, lon, altitude=None):
        """Belirtilen waypoint'e git"""
        if altitude is None:
            altitude = self.params.patrol_altitude
        location = LocationGlobalRelative(lat, lon, altitude)
        self.vehicle.simple_goto(location, groundspeed=self.params.patrol_speed)
        
    def find_nearest_waypoint_and_go(self):
        """Bir sonraki sıralı waypoint'e git"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.patrol_coordinates)
        
        next_waypoint = self.patrol_coordinates[self.current_waypoint_index]
        self.goto_waypoint(next_waypoint[0], next_waypoint[1])
        
        self.logger.info(f"📍 Sıradaki waypoint: WP{self.current_waypoint_index+1}: {next_waypoint}")
    
    def get_distance_metres(self, lat1, lon1, lat2, lon2):
        """İki GPS koordinatı arasındaki mesafeyi hesapla"""
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        return math.sqrt((dlat * 1.113195e5) ** 2 + (dlon * 1.113195e5) ** 2)

    def emergency_land(self):
        """Acil iniş prosedürü"""
        self.logger.error("🆘 Acil iniş yapılıyor!")
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(5)
        self.vehicle.close()
        if self.cap.isOpened():
            self.cap.release()
        self.running = False

    def stop(self):
        """Sistemi durdur"""
        self.running = False
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()


def main():
    # Devriye koordinatları - Yorumlarla arama durumu belirtildi
    #***************************************************************************************
    patrol_coordinates =[
    (40.7346897, 30.0883287),
    (40.7349133, 30.0857538),
    (40.7353340, 30.0858691),
    (40.7350433, 30.0861266),
    (40.7353014, 30.0864002),
    (40.7350007, 30.0866631),
    (40.7352608, 30.0869930),
    (40.7349763, 30.0871941),
    (40.7352364, 30.0874758),
    (40.7349417, 30.0876635),
    (40.7351897, 30.0879881),
    (40.7348767, 30.0882027),
    (40.7351653, 30.0884923),
    (40.7346531, 30.0885057),
    (40.7348807, 30.0857431),
    (40.7350555, 30.0857967),
    (40.7353197, 30.0861400),
    (40.7350230, 30.0864136),
    (40.7352750, 30.0867060),
    (40.7349824, 30.0869125),
    (40.7352567, 30.0872505),
    (40.7349519, 30.0874543),
    (40.7351897, 30.0877440),
    (40.7349051, 30.0878942),
    (40.7351775, 30.0882751),
    (40.7349173, 30.0884923),
    (40.7346714, 30.0885513)
]
    #***************************************************************************************
    
    patrol_drone = PatrolDrone(patrol_coordinates)
    
    try:
        while patrol_drone.running:
            time.sleep(1)
    except KeyboardInterrupt:
        patrol_drone.logger.info("🛑 Sistem kapatılıyor...")
    finally:
        patrol_drone.stop()

if __name__ == '__main__':
    main()
