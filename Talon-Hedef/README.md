<div align="center">

  # 🎯 Talon — Hedef Takip & Kamikaze Görevi
  ### KAF Technology · Teknofest Savaşan İHA 2025

  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
  ![MAVLink](https://img.shields.io/badge/MAVLink-Protocol-FF4444?style=for-the-badge)
  ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
**Talon** platformu üzerinde çalışan iki temel görev modülü:
1. **Hedef Takip** — Kamera görüntüsünden tespit edilen hedefe otonom takip
2. **Kamikaze Görevi** — Tespit edilen hedefe otonom dalış görevi

### 📂 Yapı
```
Talon-Hedef/
├── target_tracker.py     # Hedef takip ana scripti
├── kamikaze_mission.py   # Kamikaze görev scripti
├── pid_controller.py     # PID yönelim kontrolcüsü
├── mavlink_bridge.py     # MAVLink haberleşme katmanı
└── config.py             # Parametre ayarları
```

### 🚀 Kullanım

**Hedef Takip:**
```bash
python target_tracker.py --connection /dev/ttyUSB0
```

**Kamikaze Görevi:**
```bash
python kamikaze_mission.py --connection /dev/ttyUSB0 --target-gps "39.123,32.456"
```

### ⚙️ Görev Akışı

**Hedef Takip:**
```
Kamera → Tespit → PID Hesaplama → MAVLink Komut → İHA Yönelimi
```

**Kamikaze:**
```
Hedef Tespiti → Yaklaşma → Terminal Faz → Dalış → Etki
```

### ⚠️ Güvenlik
Bu kod yalnızca **yarışma ortamında** ve yetkili personel gözetiminde kullanılmak üzere tasarlanmıştır.

### Bağımlılıklar
```
pymavlink
dronekit
opencv-python
numpy
```

---

## 🇬🇧 English

### About
Two core mission modules running on the **Talon** platform:
1. **Target Tracking** — Autonomous tracking of target detected from camera feed
2. **Kamikaze Mission** — Autonomous dive mission toward the detected target

### 🚀 Usage

**Target Tracking:**
```bash
python target_tracker.py --connection /dev/ttyUSB0
```

**Kamikaze Mission:**
```bash
python kamikaze_mission.py --connection /dev/ttyUSB0 --target-gps "39.123,32.456"
```

### Mission Flow

**Target Tracking:**
```
Camera → Detection → PID Calculation → MAVLink Command → UAV Orientation
```

**Kamikaze:**
```
Target Detection → Approach → Terminal Phase → Dive → Impact
```

### ⚠️ Safety Note
This code is designed for use **only in a competition environment** and under the supervision of authorized personnel.
