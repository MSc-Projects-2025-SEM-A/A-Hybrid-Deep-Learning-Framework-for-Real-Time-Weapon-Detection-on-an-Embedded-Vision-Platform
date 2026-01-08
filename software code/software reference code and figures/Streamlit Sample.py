import streamlit as st

st.set_page_config(layout="wide")
st.title("Live Video Stream")

VIDEO_URL = "http://localhost:8000/video"

st.markdown(
    f"""
    <img src="{VIDEO_URL}" width="100%" />
    """,
    unsafe_allow_html=True
)
