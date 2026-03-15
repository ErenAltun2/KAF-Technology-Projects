from pymavlink import mavutil
import time
import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyzbar.pyzbar as pyzbar
import numpy as np

# Global değişkenler
current_agl = 0.0
current_airspeed = 0.0
agl_updated = 0
running = True
camera_subscriber = None  # Global kamera subscriber referansı

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.get_logger().info('Camera subscriber başlatıldı')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/world/talon_runway/model/mini_talon_vtail/link/base_link/sensor/talon_camera/image',
            self.image_callback,
            10
        )

        self.msg_count = 0
        self.qr_detected = False
        self.qr_data = None
        self.qr_center = None
        self.image_center = None
        self.last_image_time = 0
        self.last_qr_data = None

    def image_callback(self, msg):
        try:
            self.msg_count += 1
            self.last_image_time = time.time()

            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            h, w = cv_image.shape[:2]
            self.image_center = (w // 2, h // 2)

            self.detect_qr_code(cv_image)

            if self.qr_detected:
                cv2.rectangle(cv_image, (50, 30), (w - 50, 120), (0, 0, 255), -1)
                cv2.putText(cv_image, "QR KODU OKUNDU!", (60, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
                cv2.putText(cv_image, f"Veri: {self.qr_data}", (60, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            cv2.putText(cv_image, f"AGL: {current_agl:.1f}m", (10, h - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(cv_image, f"Hız: {current_airspeed:.1f}m/s", (10, h - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Mini Talon Camera - QR Kod Tarayıcı", cv_image)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                global running
                running = False

        except Exception as e:
            self.get_logger().error(f'Görüntü işleme hatası: {e}')

    def detect_qr_code(self, image):
        try:
            qr_codes = pyzbar.decode(image)
            for qr in qr_codes:
                qr_data = qr.data.decode('utf-8')
                points = qr.polygon
                if points and len(points) >= 4:
                    pts = [(point.x, point.y) for point in points]
                    center_x = sum(p[0] for p in pts) // len(pts)
                    center_y = sum(p[1] for p in pts) // len(pts)
                    self.qr_center = (center_x, center_y)
                    cv2.polylines(image, [np.array(pts, dtype=np.int32)], True, (0, 255, 0), 3)
                    cv2.circle(image, self.qr_center, 8, (0, 0, 255), -1)
                    cv2.circle(image, self.image_center, 8, (255, 0, 0), -1)
                    cv2.line(image, self.image_center, self.qr_center, (255, 255, 0), 2)

                if qr_data != self.last_qr_data:
                    self.qr_detected = True
                    self.qr_data = qr_data
                    self.last_qr_data = qr_data
                    print(f"🔍 QR KODU OKUNDU: {qr_data}")
                    self.get_logger().info(f'QR kod tespit edildi: {qr_data}')

        except Exception as e:
            self.get_logger().error(f'QR kod tespit hatası: {e}')

def start_ros2_camera():
    global camera_subscriber
    def ros2_thread():
        try:
            rclpy.init()
            camera_subscriber = CameraSubscriber()
            cv2.namedWindow("Mini Talon Camera - QR Kod Tarayıcı", cv2.WINDOW_AUTOSIZE)
            rclpy.spin(camera_subscriber)
        except Exception as e:
            print(f"❌ ROS2 Hata: {e}")
        finally:
            cv2.destroyAllWindows()
            if camera_subscriber:
                camera_subscriber.destroy_node()
            rclpy.try_shutdown()

    thread = threading.Thread(target=ros2_thread, daemon=True)
    thread.start()
    return thread

def main():
    global running
    print("\U0001f6e9️ TALON QR MISSION BAŞLIYOR \U0001f6e9️")
    master = mavutil.mavlink_connection('127.0.0.1:14550')
    master.wait_heartbeat()
    print("✅ MAVLink bağlantısı kuruldu.")

    start_ros2_camera()
    time.sleep(5)

    def telemetry_reader():
        global current_agl, current_airspeed, running
        while running:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg:
                current_agl = msg.relative_alt / 1000.0
            msg2 = master.recv_match(type='VFR_HUD', blocking=False)
            if msg2:
                current_airspeed = msg2.airspeed
            time.sleep(0.05)

    threading.Thread(target=telemetry_reader, daemon=True).start()

    try:
        master.set_mode_apm('FBWA')
        time.sleep(2)
        print("🟡 ARM ediliyor...")
        master.arducopter_arm()
        master.motors_armed_wait()
        print("✅ ARM başarılı")

        print("🚀 Throttle artışı...")
        for throttle in range(1100, 1801, 100):
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                1500, 1500, throttle, 1500, 0, 0, 0, 0
            )
            print(f"⚡️ Throttle: {throttle}")
            time.sleep(1)

        while current_airspeed < 15.0 and running:
            print(f"🌪️ Hız: {current_airspeed:.2f}, AGL: {current_agl:.2f}")
            time.sleep(0.5)

        print("🔼 Tırmanış başlatılıyor...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1650, 1800, 1500, 0, 0, 0, 0
        )

        while current_agl < 20.0 and running:
            print(f"📏 AGL: {current_agl:.2f}, Hız: {current_airspeed:.2f}")
            time.sleep(0.5)

        print("🌟 20 metreye ulaşıldı. Dalış başlatılıyor...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1200, 1600, 1500, 0, 0, 0, 0
        )

        while current_agl > 5.0 and running:
            print(f"📉 Dalışta - AGL: {current_agl:.2f}")
            time.sleep(0.5)

        print("🟢 5 metreye ulaşıldı. Toparlanıyor...")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            1500, 1700, 1800, 1500, 0, 0, 0, 0
        )
        time.sleep(10)

        print("🏎️ Görev tamamlandı.")
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    except KeyboardInterrupt:
        print("🛑 Kullanıcı tarafından durduruldu.")
    finally:
        running = False
        time.sleep(1)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

