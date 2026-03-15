<div align="center">

  # 🔺🔷 Drone Üçgen & Altıgen Algılama — Gerçek Drone
  ### KAF Technology · Teknofest 2025

  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
  ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
  ![Servo](https://img.shields.io/badge/Servo-Motor-FF6600?style=for-the-badge)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
Gerçek drone üzerinde çalışan **üçgen ve altıgen şekil algılama** sistemi. OpenCV ile kontur analizi yapılır, tespit edilen hedefe servo motor ile kilitlenilir.

### 📂 Yapı
```
Drone-Ucgen-Altigen-Droneda/
├── shape_detector.py     # Ana algılama scripti
├── servo_control.py      # Servo motor kontrolü
├── config.py             # Parametre ayarları
└── utils/
    ├── color_filter.py   # HSV renk filtresi
    └── geometry.py       # Şekil geometri yardımcıları
```

### 🚀 Kullanım
```bash
python shape_detector.py --camera 0 --servo-pin 18
```

### ⚙️ Konfigürasyon
```python
# config.py
TRIANGLE_SIDES = 3
HEXAGON_SIDES  = 6
HSV_LOWER = (35, 50, 50)   # Hedef renk alt sınır
HSV_UPPER = (85, 255, 255) # Hedef renk üst sınır
SERVO_PIN = 18
```

### Bağımlılıklar
```
opencv-python
numpy
RPi.GPIO   # Raspberry Pi servo kontrolü
pymavlink
```

---

## 🇬🇧 English

### About
**Triangle and hexagon shape detection** system running on a real drone. Performs contour analysis with OpenCV and locks onto the detected target using a servo motor.

### 🚀 Usage
```bash
python shape_detector.py --camera 0 --servo-pin 18
```
