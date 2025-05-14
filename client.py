# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Streaming camera frames to OpenVLA @ 20 FPS, blocking CLI prompt.
# """

import sys
import os
import requests
import json_numpy
json_numpy.patch()
import numpy as np
import time
#pip install opencv-python
import cv2
import matplotlib.pyplot as plt


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2
from convert import check_for_end_or_abort, move_to_home_position, execute_dof_action


hz=20
interval = 1/hz

def send_log(msg: str):
    url = "http://localhost:8000/log" 
    resp = requests.post(url, json={"text": msg})
    print("Server says:", resp.json())

def send_act(image: np.array, instruction: str):
    url = "http://localhost:8000/act" 
    resp = requests.post(
        url,
        json={"image": image,
              "instruction": instruction,
              "unnorm_key":"roboturk"})
    
    print("Server says:", resp.json())
    return resp.json()


def crop_center_square(image):
    h, w = image.shape[:2]
    min_dim = min(h,w)
    start_x = w//2 - min_dim//2
    start_y = h//2 - min_dim//2
    return image[start_y: start_y+min_dim, start_x: start_x + min_dim]

def capture_and_process_image():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")
    
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("failed to capture image")
    
    frame = crop_center_square(frame)
    frame_resized = cv2.resize(frame,(256,256))
    return frame_resized



def main():
    print("输入指令；按 Ctrl‑C 结束当前指令动作流；Ctrl‑D / Ctrl‑Z 退出程序。\n")
    # Import utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        sys.exit("camera can\'t open")
    cap.release()

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        print("successfully connected to kinova.")
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        move_to_home_position(base)
        
        # asking for prompt
        try:
            try:
                instruction = input("👉 instruction> ").strip()
            except (EOFError, KeyboardInterrupt):  # Ctrl‑D / Ctrl‑Z / Ctrl‑C at prompt
                print("\exit")
            print(f"execute: “{instruction}” —— Ctrl‑C to exit.\n")

            # keep updating third-person view on hz basis
            while(True):
                instruction = "do something"
                img = capture_and_process_image()
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                resp = send_act(img_rgb, instruction)
                execute_dof_action(base, base_cyclic, resp)
                time.sleep(3)     
        finally:
            cap.release()



if __name__ == "__main__":
    main()