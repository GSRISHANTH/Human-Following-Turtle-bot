"""Webcam capture in a child process.

A stuck DirectShow open holds the GIL, so a thread timeout never fires.
This process can be terminated if the first frame does not arrive.
"""

import time

import cv2

FRAME_W = 640
FRAME_H = 480


def camera_process(frame_queue):
    cap = None
    # Media Foundation delivers ~26 fps on this laptop where DirectShow stalls near 14.
    for api, name in ((cv2.CAP_MSMF, "Media Foundation"), (cv2.CAP_DSHOW, "DirectShow")):
        trial = cv2.VideoCapture(0, api)
        if trial.isOpened():
            cap = trial
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            frame_queue.put({"status": "ready", "backend": name})
            break
        trial.release()
    if cap is None:
        frame_queue.put({"status": "none"})
        return

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            time.sleep(0.05)
            continue
        try:
            if frame_queue.full():
                frame_queue.get_nowait()
            frame_queue.put(frame)
        except Exception:
            pass
