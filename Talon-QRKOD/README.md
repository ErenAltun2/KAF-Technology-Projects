<div align="center">

  # 📷 Talon — QR Kod Okuma Görevi
  ### KAF Technology · Teknofest 2025

  ![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
  ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
  ![MAVLink](https://img.shields.io/badge/MAVLink-Protocol-FF4444?style=for-the-badge)
</div>

---

## 🇹🇷 Türkçe

### Proje Hakkında
**Talon** platformu üzerinde görev sırasında QR kod tespiti, içerik çözümleme ve veriyi yerden kontrol istasyonuna iletme algoritması.

### 📂 Yapı
```
Talon-QRKOD/
├── qr_reader.py          # Ana QR okuma scripti
├── mavlink_bridge.py     # MAVLink haberleşme
├── gcs_sender.py         # Yer kontrol istasyonu gönderici
├── logger.py             # GPS-QR eşleştirme logu
└── config.py             # Parametre ayarları
```

### 🚀 Kullanım
```bash
python qr_reader.py --connection /dev/ttyUSB0 --camera 0
```

### ⚙️ Özellikler
- Uçuş sırasında gerçek zamanlı QR tespit ve çözümleme
- pyzbar + OpenCV ile çoklu QR kod desteği
- Okunan QR verisi + GPS koordinatı eşleştirmesi
- MAVLink üzerinden yer istasyonuna anlık veri iletimi
- Aynı QR'yi tekrar okumama (duplicate filter)

### Veri Formatı (Log)
```json
{
  "timestamp": "2025-09-10T10:23:45",
  "qr_data": "TARGET_ALPHA",
  "gps": { "lat": 41.123, "lon": 32.456, "alt": 25.0 }
}
```

### Bağımlılıklar
```
pyzbar
opencv-python
pymavlink
numpy
```

---

## 🇬🇧 English

### About
QR code detection, decoding, and data transmission algorithm to the ground control station during mission on the **Talon** platform.

### 🚀 Usage
```bash
python qr_reader.py --connection /dev/ttyUSB0 --camera 0
```

### Features
- Real-time QR detection and decoding during flight
- Multi-QR code support with pyzbar + OpenCV
- QR data + GPS coordinate matching
- Real-time data transmission to ground station via MAVLink
- Duplicate filter (no re-reading of the same QR)

### Log Format
```json
{
  "timestamp": "2025-09-10T10:23:45",
  "qr_data": "TARGET_ALPHA",
  "gps": { "lat": 41.123, "lon": 32.456, "alt": 25.0 }
}
```
