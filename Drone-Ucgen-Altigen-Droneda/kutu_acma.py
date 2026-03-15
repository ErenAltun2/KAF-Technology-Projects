from pymavlink import mavutil
import time

# GÜVENLİ PWM LİMİTLERİ
PWM_MIN = 1000
PWM_MAX = 2000

def angle_to_pwm(angle):
    # Açıyı önce güvenli aralıkta sınırla
    angle = max(0, min(180, angle))
    # 0° -> 1000us, 180° -> 2000us
    pwm = int(1000 + (angle / 180.0) * 1000)
    # Güvenli limitler içinde tut
    return max(PWM_MIN, min(PWM_MAX, pwm))

def send_servo(vehicle,channel, angle):
    pwm = angle_to_pwm(angle)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,     # SERVOx numarası (ör: 10)
        pwm,         # mikro-saniye
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"[SERVO] CH{channel} -> {angle}° ({pwm}us)")

def disable_servo(vehicle, channel):
    """Servo çıkışını devre dışı bırak"""
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        0,  # PWM = 0 (devre dışı)
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"[SERVO] CH{channel} -> DISABLED")

def box_open(vehicle,channel, wait=2.0,degree=180):
    # Kutu kapağını AÇ - güvenli açı limiti ile
    safe_degree = min(degree, 180)
    if degree > 180:
        print(f"UYARI: {degree}° yerine güvenli {safe_degree}° kullanılıyor")
    
    send_servo(vehicle,channel, safe_degree)
    time.sleep(wait)
    # Servo'yu devre dışı bırak
    disable_servo(vehicle, channel)

def box_close(vehicle,channel, wait=1.0,degree=0):
    # Kutu kapağını KAPAT
    send_servo(vehicle,channel, degree)
    time.sleep(wait)
    # Servo'yu devre dışı bırak
    disable_servo(vehicle, channel)
