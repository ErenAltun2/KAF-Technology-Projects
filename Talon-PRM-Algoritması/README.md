<div align="center">

  # 🗺️ Talon — PRM Yol Planlama Algoritması
  ### KAF Technology · Teknofest 2025

  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
  ![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros&logoColor=white)
  ![NetworkX](https://img.shields.io/badge/NetworkX-Graph-orange?style=for-the-badge)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
**Probabilistic Roadmap Method (PRM)** — Talon platformu için engelli ortamlarda otonom yol planlama algoritması. Rastgele örnekleme ile çevreyi keşfeder, A* ile optimal rotayı hesaplar.

### 📂 Yapı
```
Talon-PRM-Algoritması/
├── prm_planner.py        # Ana PRM algoritması
├── astar.py              # A* yol bulma
├── collision_checker.py  # Çarpışma kontrolü
├── visualizer.py         # Yol görselleştirme
├── maps/                 # Yarışma haritaları (.yaml)
└── config.py             # PRM parametreleri
```

### 🚀 Kullanım
```bash
python prm_planner.py \
  --map maps/competition_map.yaml \
  --start "0,0,15" \
  --goal "200,150,20"
```

### ⚙️ Algoritma Parametreleri
```python
# config.py
NUM_SAMPLES    = 500    # Örnekleme noktası sayısı
K_NEIGHBORS    = 10     # K-en yakın komşu
STEP_SIZE      = 5.0    # Adım büyüklüğü (metre)
SAFETY_MARGIN  = 3.0    # Engel güvenlik mesafesi (metre)
MAX_ALTITUDE   = 50.0   # Maksimum uçuş irtifası (metre)
```

### Algoritma Akışı
```
Harita Yükle → Rastgele Örnekle → Çarpışma Kontrol → Graf Oluştur → A* ile Rota Bul → Waypoint Gönder
```

### Bağımlılıklar
```
numpy
scipy
networkx
matplotlib
pyyaml
pymavlink
```

---

## 🇬🇧 English

### About
**Probabilistic Roadmap Method (PRM)** — autonomous path planning algorithm for the Talon platform in environments with obstacles. Explores the environment through random sampling and calculates the optimal route using A*.

### 🚀 Usage
```bash
python prm_planner.py \
  --map maps/competition_map.yaml \
  --start "0,0,15" \
  --goal "200,150,20"
```

### Algorithm Parameters
```python
NUM_SAMPLES    = 500    # Number of sampling points
K_NEIGHBORS    = 10     # K-nearest neighbors
STEP_SIZE      = 5.0    # Step size (meters)
SAFETY_MARGIN  = 3.0    # Obstacle safety margin (meters)
```

### Algorithm Flow
```
Load Map → Random Sample → Collision Check → Build Graph → Find Route with A* → Send Waypoints
```
