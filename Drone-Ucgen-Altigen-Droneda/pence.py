import time
from pymavlink import mavutil
import kutu_acma

def send_pwm(vehicle, channel, pwm_value):
    """
    DroneKit vehicle nesnesi üzerinden belirtilen servoya PWM gönderir.
    """
    # PWM değerini güvenli limitler içinde tut
    if pwm_value != 0:  # 0 devre dışı bırakma için
        pwm_value = max(1000, min(2000, pwm_value))
    
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm_value,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"[SERVO] CH{channel} -> {pwm_value}us")

def rotate_sequence(vehicle, channel):
    """
    Servo ile sağa dön, dur, sola dön, dur hareketini uygular.
    Süreler ve PWM değerleri ihtiyaca göre değiştirilebilir.
    """
    PWM_STOP = 0
    PWM_RIGHT = 1700
    PWM_LEFT = 1300

    # Sağa dönüş
    send_pwm(vehicle, channel, PWM_RIGHT)
    time.sleep(3.83)

    # Durdur
    send_pwm(vehicle, channel, PWM_STOP)
    time.sleep(5)

    
    kutu_acma.box_open(vehicle=vehicle,channel=10, wait=2.0,degree=180)  # 200 yerine 180
    kutu_acma.box_close(vehicle=vehicle,channel=10,wait=2.0,degree=0 )

    # Sola dönüş
    send_pwm(vehicle, channel, PWM_LEFT)
    time.sleep(3.83)

    # Durdur
    send_pwm(vehicle, channel, PWM_STOP)
    
    # Servo'yu devre dışı bırak
    kutu_acma.disable_servo(vehicle, channel)
