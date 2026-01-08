import streamlit as st
import requests
from PIL import Image
import io
import time

st.set_page_config(layout="wide")
st.title("🔴 Live Gun Detection (ESP32 → YOLO → Streamlit)")

FRAME_URL = "http://localhost:5000/latest_frame"

frame_box = st.empty()
status = st.empty()

while True:
    try:
        r = requests.get(FRAME_URL, timeout=2)

        if r.status_code == 200:
            img = Image.open(io.BytesIO(r.content))
            frame_box.image(img, channels="RGB", use_container_width=True)
            status.success("Live feed running")
        else:
            status.warning("No frame received yet")

    except:
        status.error("Waiting for server...")

    time.sleep(0.15)
