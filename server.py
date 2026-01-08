from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from ultralytics import YOLO
from PIL import Image, ImageDraw, ImageFont
import io
import os
import numpy as np
import time
import base64

# ---------------- CONFIG ----------------
MODEL_PATH = os.environ.get("YOLO_MODEL_PATH", "gun.pt")
CONF_THRESHOLD = float(os.environ.get("CONF_THRESHOLD", 0.5))

# ---------------- APP ----------------
app = Flask(__name__)
CORS(app)

print("Loading YOLO model:", MODEL_PATH)
model = YOLO(MODEL_PATH)
print("Model loaded")

# ---------------- FRAME CACHE ----------------
LAST_ANNOTATED_FRAME = None
LAST_FRAME_TIME = None

# ---------------- HELPERS ----------------
def pil_to_np_bgr(pil_img):
    rgb = np.array(pil_img)
    if rgb.ndim == 2:
        rgb = np.stack((rgb,) * 3, axis=-1)
    return rgb[:, :, ::-1]


def run_yolo_on_pil(pil_img):
    img_np = pil_to_np_bgr(pil_img)
    start = time.time()
    results = model(img_np, conf=CONF_THRESHOLD)
    infer_time = time.time() - start

    detections = []
    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            name = model.names.get(cls_id, str(cls_id))
            detections.append({
                "class": name,
                "conf": conf,
                "bbox": [x1, y1, x2, y2]
            })

    return detections, infer_time


def annotate_image(pil_img, detections):
    draw = ImageDraw.Draw(pil_img)
    try:
        font = ImageFont.load_default()
    except:
        font = None

    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        label = f"{det['class']} {det['conf']:.2f}"
        draw.rectangle([x1, y1, x2, y2], outline="red", width=2)
        draw.text((x1, max(0, y1 - 12)), label, fill="red", font=font)

    buf = io.BytesIO()
    pil_img.save(buf, format="JPEG")
    buf.seek(0)
    return buf.getvalue()

# ---------------- ENDPOINTS ----------------
@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


@app.route("/predict_annotated", methods=["POST"])
def predict_annotated():
    global LAST_ANNOTATED_FRAME, LAST_FRAME_TIME

    if "image" in request.files:
        img = Image.open(request.files["image"].stream).convert("RGB")
    else:
        data = request.get_json(silent=True) or {}
        b64 = data.get("image_base64")
        if not b64:
            return jsonify({"error": "No image"}), 400
        if "," in b64:
            b64 = b64.split(",", 1)[1]
        img = Image.open(io.BytesIO(base64.b64decode(b64))).convert("RGB")

    detections, infer_time = run_yolo_on_pil(img)
    annotated = annotate_image(img.copy(), detections)

    LAST_ANNOTATED_FRAME = annotated
    LAST_FRAME_TIME = time.time()

    print(f"Frame processed | {infer_time:.2f}s | Detections: {len(detections)}")

    return send_file(io.BytesIO(annotated), mimetype="image/jpeg")


@app.route("/latest_frame", methods=["GET"])
def latest_frame():
    if LAST_ANNOTATED_FRAME is None:
        return jsonify({"status": "no frame yet"}), 404

    return send_file(
        io.BytesIO(LAST_ANNOTATED_FRAME),
        mimetype="image/jpeg"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
