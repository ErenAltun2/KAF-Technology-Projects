from flask import Flask, Response
import cv2

app = Flask(__name__)

print("📹 USB Kamera başlatılıyor...")
camera = cv2.VideoCapture(0)  # 0: default USB kamera index
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not camera.isOpened():
    print("❌ USB kamera açılamadı")
else:
    print("✅ USB kamera başlatıldı")


def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            continue

        # Görüntüyü JPEG formatına çevir
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        if not ret:
            continue

        frame = buffer.tobytes()

        # MJPEG stream oluştur
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    # 0.0.0.0 -> ağdaki herkes bağlanabilsin
    app.run(host="0.0.0.0", port=5000)

