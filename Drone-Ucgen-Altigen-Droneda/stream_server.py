import threading
import time
import cv2
from flask import Flask, Response

_app = Flask(__name__)
_latest_jpeg = None
_lock = threading.Lock()

_last_push = 0.0
_target_fps = 8.0  # 8–10 fps Pi 4 için iyi bir üst limit
_min_interval = 1.0 / _target_fps

# Yayın çözünürlüğü 
STREAM_W, STREAM_H = 320, 240

def update_frame(bgr_image):
    """
    Ana koddan gelen BGR frame'i küçült, (opsiyonel griye çevir), 
    düşük kalitede JPEG'e encode et ve paylaştır.
    """
    global _latest_jpeg, _last_push

    now = time.time()
    if now - _last_push < _min_interval:
        return  # FPS sınırlama: bu frame'i atla

    # Küçük çözünürlük: encode öncesi downscale en çok CPU kurtarır
    small = cv2.resize(bgr_image, (STREAM_W, STREAM_H), interpolation=cv2.INTER_AREA)

    # DÜŞÜK JPEG kalite (20–40 arası oldukça ucuz)
    ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
    if ok:
        with _lock:
            _latest_jpeg = buf.tobytes()
        _last_push = now

@_app.route("/video")
def video():
    def gen():
        boundary = b"--frame\r\n"
        while True:
            with _lock:
                frame = _latest_jpeg
            if frame is not None:
                yield boundary
                yield b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.02)  # çok küçük bir bekleme (I/O için)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

def start_server(host="0.0.0.0", port=5001):
    t = threading.Thread(
        target=_app.run, 
        kwargs={"host": host, "port": port, "threaded": True, "use_reloader": False},
        daemon=True
    )
    t.start()
