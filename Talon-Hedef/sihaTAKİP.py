from ultralytics import YOLO
import cv2
import torch
from dronekit import connect, VehicleMode
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque
import math

# MAVLink bağlantısını simüle et
connection_string = "udp:127.0.0.1:14550"  # Simülasyon bağlantısı
master = connect(connection_string)
# Modeli yükle
model_path = "best.pt"
model = YOLO(model_path)

# Cihazı ayarla
device = "cuda" if torch.cuda.is_available() else "cpu"
model.to(device)

# Video bilgileri
frame_width = 640  # Genişlik
frame_height = 480  # Yükseklik

center_x = frame_width //2
center_y = frame_height //2

# RC kanal ayar fonksiyonu
def set_rc_channels(aileron, elevator, throttle):
    master.channels.overrides = {
        '1': aileron,
        '2': elevator,
        '3': throttle,
    }

# Görselleştirme için ekran ortasında yeşil dikdörtgen (opsiyonel)
rectangle_width = int(frame_width * 0.50)
rectangle_height = int(frame_height * 0.80)
rect_x1 = (frame_width - rectangle_width) // 2
rect_y1 = (frame_height - rectangle_height) // 2
rect_x2 = rect_x1 + rectangle_width
rect_y2 = rect_y1 + rectangle_height
rectangle_color = (0,0,255)  # mavi

# Uçak sınıfının ID'si
airplane_class_id = 0  # Modelin eğitildiği sınıfa göre değişebilir

# Mesafe eşikleri (piksel cinsinden)
max_distance_threshold = 150 

# Önceki frame'deki x merkezi
target_previous_x = None


class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.integral_limit = integral_limit  # Integral bileşenini sınırlamak için

    def compute(self, error):
        self.integral += error
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))  # Sınırlama
        derivative = error - self.prev_error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output


def adaptive_pid_tuning(error, pid_controller):
    if abs(error) > 100:
        pid_controller.kp = 0.8  # Büyük hata -> Daha agresif düzeltme
        pid_controller.ki = 0.05
        pid_controller.kd = 0.4
    elif abs(error) > 50:
        pid_controller.kp = 0.6  # Orta hata -> Normal PID
        pid_controller.ki = 0.04
        pid_controller.kd = 0.3
    else:
        pid_controller.kp = 0.3  # Küçük hata -> Hafif düzeltme
        pid_controller.ki = 0.02
        pid_controller.kd = 0.2

dead_zone = 10  # Ölü bölge
queue_size = 5
x_history = deque(maxlen=queue_size)
y_history = deque(maxlen=queue_size)


aileron_pid = PIDController(0.5, 0.04, 0.3)  # X ekseni için (Sağa-Sola)
elevator_pid = PIDController(0.5, 0.04, 0.3)  # Y ekseni için (Yukarı-Aşağı)


import math


# Uçak takip kısmında adaptif PID'yi kullanma
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 'camera/image', self.listener_callback, 10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        global target_previous_x
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        res = 640
        resized_frame = cv2.resize(frame, (res, res))
        frame_tensor = torch.from_numpy(resized_frame).permute(2, 0, 1).float()
        frame_tensor = frame_tensor.unsqueeze(0).to(device) / 255.0


        font = cv2.FONT_HERSHEY_SIMPLEX
        text = "KAF"  # Sadece "KAF" yazısı
        text_size = cv2.getTextSize(text, font, 0.7, 2)[0]  # Font boyutunu 0.7 olarak ayarladım
        text_x = frame.shape[1] - text_size[0] - 10  # Sağ üst köşe için X konumu
        text_y = 30  # Üstten 30 px uzaklık
        cv2.putText(frame, text, (text_x, text_y), font, 0.7, (59,41,251), 2, cv2.LINE_AA)
        


        results = model(frame_tensor, verbose=False)
        # Opsiyonel: Görselleştirme için yeşil dikdörtgen
        cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), rectangle_color, 2)
        
        aileron_value = 1500
        elevator_value = 1500
        throttle_value = 1500
        
        
        if results and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                confidence = box.conf[0]
                
                if class_id == airplane_class_id:
                    class_name = model.names[class_id]
                    scale_x = frame_width / res
                    scale_y = frame_height / res
                    x1, x2 = int(x1 * scale_x), int(x2 * scale_x)
                    y1, y2 = int(y1 * scale_y), int(y2 * scale_y)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    object_center_x = (x1 + x2) // 2
                    object_center_y = (y1 + y2) // 2
                    error_x = object_center_x - (frame_width // 2)
                    error_y = object_center_y - (frame_height // 2)
                    cv2.line(frame, (center_x, center_y), (object_center_x, object_center_y), (255,0,0), 2)
                    # Adaptif PID parametrelerini her hata için ayarla
                    adaptive_pid_tuning(error_x, aileron_pid)  # X ekseni için adaptif PID
                    adaptive_pid_tuning(error_y, elevator_pid)  # Y ekseni için adaptif PID

                    x_history.append(object_center_x)
                    y_history.append(object_center_y)
                    object_center_x = int(sum(x_history) / len(x_history))
                    object_center_y = int(sum(y_history) / len(y_history))


                    target_previous_x = object_center_x  # Güncelle
                    aileron_value = max(1200, min(1900, aileron_value + int(aileron_pid.compute(error_x))))
                    elevator_value = max(1200, min(1900, elevator_value - int(elevator_pid.compute(error_y))))

                    # Throttle ayarlamaları
                    dikdortgen_alani = (x2 - x1) * (y2 - y1)
                    base_throttle = 1500
                    min_throttle = 1000
                    max_throttle = 2000
                    lower_threshold = 500
                    upper_threshold = 1300

                    if dikdortgen_alani < lower_threshold:
                        diff_percentage = (lower_threshold - dikdortgen_alani) / lower_threshold
                        throttle_value = base_throttle + diff_percentage * (max_throttle - base_throttle)
                        throttle_value = min(int(throttle_value), max_throttle)
                    elif dikdortgen_alani > upper_threshold:
                        diff_percentage = (dikdortgen_alani - upper_threshold) / upper_threshold
                        throttle_value = base_throttle - diff_percentage * (base_throttle - min_throttle)
                        throttle_value = max(int(throttle_value), min_throttle)
                    else:
                        throttle_value = base_throttle

            set_rc_channels(
                aileron_value,
                elevator_value,
                throttle_value,
            )
            print(f"aileron{aileron_value}")
            print(f"elevator{elevator_value}")
            print(f"throttle{throttle_value}")
            

        cv2.imshow("Airplane Tracking Simulation", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    master.close()

if __name__ == '__main__':
    main()
