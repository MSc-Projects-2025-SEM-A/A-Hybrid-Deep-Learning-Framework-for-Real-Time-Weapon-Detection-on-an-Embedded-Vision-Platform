import cv2
import requests
import time

SERVER_URL = "http://localhost:5000/predict_annotated"
CAMERA_ID = 0
FRAME_DELAY = 0.2  # ~5 FPS
JPEG_QUALITY = 80

def main():
    cap = cv2.VideoCapture(CAMERA_ID)

    if not cap.isOpened():
        print("❌ Webcam not found")
        return

    print("✅ ESP32 Simulator started (Webcam → Server)")
    print("📡 Sending frames to:", SERVER_URL)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (320, 240))

        _, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )

        files = {
            "image": ("frame.jpg", jpeg.tobytes(), "image/jpeg")
        }

        try:
            start = time.time()
            r = requests.post(SERVER_URL, files=files, timeout=5)
            print(f"Frame sent | RTT: {time.time() - start:.2f}s")
        except Exception as e:
            print("⚠️ Error:", e)

        time.sleep(FRAME_DELAY)

    cap.release()

if __name__ == "__main__":
    main()
