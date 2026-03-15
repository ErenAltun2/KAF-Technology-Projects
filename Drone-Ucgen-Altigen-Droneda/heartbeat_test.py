from pymavlink import mavutil
import time

PORT = '/dev/serial0'
BAUD = 57600 # Pixhawk baudrate ile aynı

try:
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    print(f"Waiting for heartbeat on {PORT} at {BAUD} baud...")

    start = time.time()
    timeout = 30  # 30 saniye bekle
    heartbeat_received = False

    while time.time() - start < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.get_srcSystem() != 0:
            heartbeat_received = True
            print(f"✅ Heartbeat received from system {msg.get_srcSystem()}, component {msg.get_srcComponent()}")
            break

    if not heartbeat_received:
        print("❌ No heartbeat received. Check cables, TELEM1 port, and baudrate!")

except Exception as e:
    print(f"❌ Error connecting: {e}")
