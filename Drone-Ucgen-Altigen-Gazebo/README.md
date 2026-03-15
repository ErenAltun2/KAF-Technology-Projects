<div align="center">

  # 🔺🔷 Drone Üçgen & Altıgen Algılama — Gazebo Simülasyon
  ### KAF Technology · Teknofest 2025

  ![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?style=for-the-badge&logo=ros&logoColor=white)
  ![Gazebo](https://img.shields.io/badge/Gazebo-11-FF6600?style=for-the-badge)
  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
ROS Noetic + Gazebo 11 ortamında drone kamerası ile **üçgen ve altıgen** şekil algılama simülasyonu. Gerçek uçuş öncesinde algoritmaların sanal ortamda test edilmesi için kullanılır.

### 📂 Yapı
```
Drone-Ucgen-Altigen-Gazebo/
├── ros_ws/
│   └── src/
│       └── shape_detection/
│           ├── launch/
│           │   └── simulation.launch
│           ├── scripts/
│           │   └── shape_detector_node.py
│           ├── worlds/
│           │   └── shape_world.world
│           └── CMakeLists.txt
└── README.md
```

### 🚀 Kurulum ve Çalıştırma
```bash
# ROS workspace kur
cd Drone-Ucgen-Altigen-Gazebo/ros_ws
catkin_make
source devel/setup.bash

# Simülasyonu başlat
roslaunch shape_detection simulation.launch
```

### ROS Topic'leri
| Topic | Tip | Açıklama |
|-------|-----|----------|
| `/camera/image_raw` | sensor_msgs/Image | Drone kamera görüntüsü |
| `/shape_detection/result` | std_msgs/String | Tespit sonucu |
| `/shape_detection/bbox` | geometry_msgs/Point | Hedef koordinatı |

---

## 🇬🇧 English

### About
Shape detection simulation for **triangles and hexagons** using drone camera in ROS Noetic + Gazebo 11. Used to test algorithms in a virtual environment before real flight.

### 🚀 Setup & Run
```bash
cd Drone-Ucgen-Altigen-Gazebo/ros_ws
catkin_make
source devel/setup.bash
roslaunch shape_detection simulation.launch
```

### Dependencies
- ROS Noetic
- Gazebo 11
- opencv-python
- cv_bridge
