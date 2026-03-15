<div align="center">

  # 🔺 Drone Üçgen Algılama — YOLO Model
  ### KAF Technology · Teknofest 2025

  ![YOLOv8](https://img.shields.io/badge/YOLOv8-Detection-00FFAA?style=for-the-badge)
  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
Drone kamerası görüntüsünden **üçgen şekillerin** gerçek zamanlı tespiti için eğitilmiş YOLO modelini içerir. Teknofest İHA yarışması şekil algılama görevi için geliştirilmiştir.

### 📂 Yapı
```
Drone-Ucgen-Algılama-Yolo-Model/
├── model/           # Eğitilmiş YOLO ağırlıkları (.pt)
├── dataset/         # Eğitim veri seti (YOLO formatı)
│   ├── images/
│   └── labels/
├── train.py         # Model eğitim scripti
├── detect.py        # Gerçek zamanlı tespit scripti
└── data.yaml        # Dataset konfigürasyonu
```

### 🚀 Kullanım
```bash
# Gerçek zamanlı kamera tespiti
python detect.py --source 0 --weights model/best.pt

# Video üzerinde test
python detect.py --source video.mp4 --weights model/best.pt

# Model eğitimi
python train.py --data data.yaml --epochs 100
```

---

## 🇬🇧 English

### About
Contains the trained YOLO model for real-time **triangle shape detection** from drone camera feed. Developed for the Teknofest UAV competition shape detection mission.

### 🚀 Usage
```bash
# Real-time camera detection
python detect.py --source 0 --weights model/best.pt

# Test on video
python detect.py --source video.mp4 --weights model/best.pt

# Train model
python train.py --data data.yaml --epochs 100
```

### Dependencies
```
ultralytics
opencv-python
torch
numpy
```
