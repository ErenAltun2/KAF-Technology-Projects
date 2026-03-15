# test_sequence.py
from dronekit import connect
import kutu_acma
import pence  # rotate_sequence burada

class ServoTest:
    def __init__(self):
        print("Pixhawk'a bağlanılıyor...")
        self.vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)
        print("Bağlantı OK")

    def run_sequence(self):
        print("=== Kutu AÇ ===")
        kutu_acma.box_open(self.vehicle, channel=9, degree=180, wait=2.0)

        print("=== Kutu KAPA ===")
        kutu_acma.box_close(self.vehicle, channel=9, degree=0, wait=2.0)

        print("=== Pence Ac ===")
        kutu_acma.box_open(self.vehicle, channel=10, degree=180, wait=2.0)

        print("=== Pence KAPA ===")
        kutu_acma.box_close(self.vehicle, channel=10, degree=0, wait=2.0)


        print("=== Pençe sekansı (sağ-dur-sol-dur) ===")
        pence.rotate_sequence(self.vehicle, channel=11)

        print("✅ Sekans tamamlandı.")

if __name__ == "__main__":
    tester = ServoTest()  
    try:
        tester.run_sequence()
    finally:
        print("bitti")
