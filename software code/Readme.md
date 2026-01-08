This folder consists of code files related to software part of my project


............................................................................................................
# A Hybrid Deep Learning Framework for Real-Time Weapon Detection on an Embedded Vision Platform

This repository contains the implementation of an **Automated Real-Time Gun Detection System** using a **YOLOv10 deep learning model** deployed across **edge devices (ESP32-CAM)** and a **Python-based inference server**. The system supports **image-based detection**, **live webcam detection**, and **real-time ESP32 streaming** with a **Streamlit dashboard**.

---

## 📌 Project Features

- YOLOv10-based gun detection (`gun.pt`)
- Image inference
- Live webcam detection
- ESP32-CAM → Flask server → YOLO inference
- Real-time annotated stream
- Streamlit monitoring dashboard
- Detection logging with timestamps

---

## 📂 Project Structure

```text
.
├── gun.pt                     # Trained YOLOv10 gun detection model
├── infer.py                   # Image / webcam inference script
├── server.py                  # Flask inference server (ESP32 compatible)
├── app.py                     # Streamlit live dashboard
├── requirements.txt           # Python dependencies
├── FIRMWARE.txt               # ESP32-CAM firmware (Arduino)
├── guns-detection.ipynb       # Training / experimentation notebook
└── README.md                  # Project documentation
```

---

## ⚙️ System Requirements

### Software

- Python **3.8 – 3.12**
- pip
- Arduino IDE (for ESP32)
- ESP32-CAM (AI Thinker)

### Hardware (Optional)

- ESP32-CAM module
- USB-to-TTL programmer
- Webcam (for local testing)

---

## 📦 Installation

### 1️⃣ Create Virtual Environment (Recommended)

```bash
python -m venv venv
source venv/bin/activate        # Linux / Mac
venv\Scripts\activate           # Windows
```

### 2️⃣ Install Dependencies

```bash
pip install -r requirements.txt
```

Dependencies are defined in `requirements.txt`, including **Ultralytics YOLOv10**, PyTorch, OpenCV, Flask, and Streamlit .

---

## 🚀 Running the System

---

## 🔹 Option 1: Image Inference

Run gun detection on a single image.

```bash
python infer.py --model gun.pt --source path/to/image.jpg --output output.jpg
```

**Arguments**

- `--model` → Path to trained model (`gun.pt`)
- `--source` → Image path
- `--output` → (Optional) Save annotated output
- `--conf` → (Optional) Confidence threshold (default `0.5`)

📌 Output:

- Annotated image
- Console logs with inference time

Code reference: `infer.py`

---

## 🔹 Option 2: Webcam Detection (Live)

```bash
python infer.py --model gun.pt --source webcam
```

📌 Features:

- Real-time detection
- FPS display
- Bounding boxes + confidence
- Logs saved to `results.txt`

---

## 🔹 Option 3: Flask Inference Server (ESP32 / API)

Start the backend server:

```bash
python server.py
```

Server runs at:

```
http://0.0.0.0:5000
```

### Available Endpoints

| Endpoint             | Method | Description                     |
| -------------------- | ------ | ------------------------------- |
| `/health`            | GET    | Server status                   |
| `/predict_annotated` | POST   | Image → YOLO → annotated output |
| `/latest_frame`      | GET    | Latest processed frame          |

Server loads `gun.pt` automatically on startup .

---

## 🔹 Option 4: ESP32-CAM Integration

1. Open **Arduino IDE**
2. Paste firmware from `FIRMWARE.txt`
3. Update:

   ```cpp
   const char* ssid = "YOUR_WIFI";
   const char* password = "YOUR_PASSWORD";
   const char* serverUrl = "http://YOUR_PC_IP:5000/predict_annotated";
   ```

4. Select **AI Thinker ESP32-CAM**
5. Upload firmware

ESP32 captures frames and streams them to the Flask server for real-time inference .

---

## 🔹 Option 5: Streamlit Live Dashboard

Start dashboard:

```bash
streamlit run app.py
```

Dashboard URL:

```
http://localhost:8501
```

📌 Displays:

- Live annotated feed from `/latest_frame`
- Server status updates

Dashboard code reference .

---

## 📊 Output & Logging

- Bounding boxes labeled **Gun**
- Confidence scores
- Inference time per frame
- Detection logs:

  ```text
  source, class, confidence, x1, y1, x2, y2, timestamp
  ```
