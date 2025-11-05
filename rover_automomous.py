#!/usr/bin/env python3
"""
rover_autonomous.py
Jetson <-> Pico (USB serial) + camera “describe, then go” wander loop.

Behavior:
- Drive in a direction.
- Poll ultrasonic distance via "DIST?" from the Pico.
- If obstacle closer than threshold -> rotate randomly and try again.
- With random chance, stop, look around (optional servo sweep), grab a frame, ask OpenAI Vision to describe it,
  speak/print the caption, then continue.

Reference command style matches your prior test script (STRAFE_L / STRAFE_R / ROT_R / STOP / DIST?).
If your Pico uses other commands (e.g., FWD/BACK or DRIVE vx vy w), adjust COMMANDS below.

Requires:
    pip install pyserial opencv-python openai
And set:
    export OPENAI_API_KEY="YOUR_KEY"
"""

import os
import sys
import time
import math
import base64
import random
import argparse
from dataclasses import dataclass
from typing import Optional

import serial
import cv2
from openai import OpenAI

# ---------- Config ----------
SERIAL_PORT       = "/dev/ttyACM0"
BAUD              = 115200
SERIAL_TIMEOUT_S  = 1.0

# If your rover is “sideways-forward” (mecanum set so STRAFE acts like forward), leave SIDEWAYS_MODE=True.
# If you have a true forward command (e.g., "FWD"), set SIDEWAYS_MODE=False and adjust COMMANDS below.
SIDEWAYS_MODE     = True

# Distance (cm) considered "too close" -> avoidance
OBSTACLE_CM       = 35.0

# Drive timing (seconds)
STEP_DRIVE_S      = 1.2    # base drive burst
STEP_ROTATE_S     = 0.6    # base rotate burst (tune for ~30–60°)

# Random stop odds per loop iteration
RANDOM_STOP_PROB  = 0.10   # 10% chance each cycle to stop and describe

# Camera
USE_CSI_CAMERA    = True   # set False to use a USB webcam at /dev/video0
CSI_SENSOR_ID     = 0
CAPTURE_W, CAPTURE_H = 1280, 720

# Vision
OPENAI_MODEL      = "gpt-4o-mini"
VISION_PROMPT     = "Describe the scene succinctly. Note key objects, obstacles, and any navigational hints."

# Optional: servo look-around (set to None to disable)
# Use a servo channel your Pico expects (your test file showed 'SERVO 15 <angle>' as an example).
SERVO_CHANNEL     = 15
SERVO_SWEEP_ANGLES = [60, 100, 80]  # quick sweep while stopped

# ---------- Command Map ----------
# Tweak these strings to match your Pico firmware.
COMMANDS = {
    # Movement
    "FORWARD":  "STRAFE_R" if SIDEWAYS_MODE else "FWD",
    "BACK":     "STRAFE_L" if SIDEWAYS_MODE else "BACK",
    "ROT_L":    "ROT_L",
    "ROT_R":    "ROT_R",
    "STOP":     "STOP",

    # Sensors
    "DIST?":    "DIST?",

    # Servo (expects: SERVO <ch> <angle_degrees>)
    "SERVO":    "SERVO",
}

# ---------- Helpers ----------
def gstreamer_csi_pipeline(sensor_id=0, capture_w=1280, capture_h=720, display_w=None, display_h=None,
                           framerate=30, flip_method=0):
    # Good default CSI pipeline for Jetson
    display_w = display_w or capture_w
    display_h = display_h or capture_h
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={capture_w}, height={capture_h}, "
        f"format=NV12, framerate={framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={display_w}, height={display_h}, format=BGRx ! "
        f"videoconvert ! appsink drop=true"
    )

def open_camera():
    if USE_CSI_CAMERA:
        cap = cv2.VideoCapture(gstreamer_csi_pipeline(CSI_SENSOR_ID, CAPTURE_W, CAPTURE_H), cv2.CAP_GSTREAMER)
    else:
        cap = cv2.VideoCapture(0)  # USB cam
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
    if not cap.isOpened():
        raise RuntimeError("Could not open camera.")
    return cap

def frame_to_base64_jpeg(frame, quality=85, max_side=1024):
    h, w = frame.shape[:2]
    scale = max_side / max(h, w)
    if scale < 1.0:
        frame = cv2.resize(frame, (int(w*scale), int(h*scale)))
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("Failed to encode JPEG.")
    return base64.b64encode(buf.tobytes()).decode("ascii")

def openai_describe_b64jpeg(b64jpg: str, prompt: str, model: str) -> str:
    client = OpenAI()
    # Simple Vision content call
    msg = client.chat.completions.create(
        model=model,
        messages=[{
            "role": "user",
            "content": [
                {"type":"input_text","text": prompt},
                {"type":"input_image","image_url":{"url": f"data:image/jpeg;base64,{b64jpg}" }}
            ]
        }],
        temperature=0.2,
        max_tokens=150
    )
    return msg.choices[0].message.content.strip()

# ---------- Pico Serial ----------
class PicoLink:
    def __init__(self, port: str, baud: int, timeout_s: float):
        self.ser = serial.Serial(port, baud, timeout=timeout_s)
        self.ser.reset_input_buffer()
        print(f"[Serial] Connected on {port} @ {baud}")

    def send(self, cmd: str) -> str:
        self.ser.write((cmd.strip() + "\n").encode())
        # Give Pico a moment if needed
        time.sleep(0.05)
        line = self.ser.readline().decode(errors="ignore").strip()
        return line

    def dist_cm(self) -> Optional[float]:
        resp = self.send(COMMANDS["DIST?"])
        # Expect formats like: "DIST: 123.4 cm" or "DIST: NONE"
        if resp.startswith("DIST:") and "NONE" not in resp:
            try:
                val = float(resp.split()[1])
                return val
            except Exception:
                return None
        return None

    def stop(self):
        self.send(COMMANDS["STOP"])

    def forward_burst(self, secs: float):
        t_end = time.time() + secs
        while time.time() < t_end:
            self.send(COMMANDS["FORWARD"])
            time.sleep(0.05)

    def back_burst(self, secs: float):
        t_end = time.time() + secs
        while time.time() < t_end:
            self.send(COMMANDS["BACK"])
            time.sleep(0.05)

    def rotate_right_burst(self, secs: float):
        t_end = time.time() + secs
        while time.time() < t_end:
            self.send(COMMANDS["ROT_R"])
            time.sleep(0.05)

    def rotate_left_burst(self, secs: float):
        t_end = time.time() + secs
        while time.time() < t_end:
            self.send(COMMANDS["ROT_L"])
            time.sleep(0.05)

    def servo(self, ch: int, angle_deg: float):
        self.send(f"{COMMANDS['SERVO']} {ch} {angle_deg:.1f}")

# ---------- Main Wander Logic ----------
def random_rotate(pico: PicoLink):
    # Randomly choose left/right and a “burst” count
    turn_left = random.random() < 0.5
    # Randomize rotation time a bit
    secs = STEP_ROTATE_S * random.uniform(0.8, 1.6)
    if turn_left:
        pico.rotate_left_burst(secs)
    else:
        pico.rotate_right_burst(secs)

def look_and_describe(pico: PicoLink, cap):
    # Stop motors
    pico.stop()
    time.sleep(0.2)

    # Optional quick servo sweep
    if SERVO_CHANNEL is not None and SERVO_SWEEP_ANGLES:
        for ang in SERVO_SWEEP_ANGLES:
            pico.servo(SERVO_CHANNEL, ang)
            time.sleep(0.3)

    # Grab one frame
    ok, frame = cap.read()
    if not ok:
        print("[Vision] Failed to capture frame.")
        return

    b64jpg = frame_to_base64_jpeg(frame)
    try:
        desc = openai_describe_b64jpeg(b64jpg, VISION_PROMPT, OPENAI_MODEL)
    except Exception as e:
        print(f"[Vision] OpenAI error: {e}")
        return

    print("\n[Scene Description]")
    print(desc)
    print()

def main():
    parser = argparse.ArgumentParser(description="Autonomous wander + random describe loop")
    parser.add_argument("--port", "-p", default=SERIAL_PORT)
    parser.add_argument("--baud", "-b", type=int, default=BAUD)
    parser.add_argument("--obstacle_cm", type=float, default=OBSTACLE_CM)
    parser.add_argument("--run_seconds", type=int, default=0, help="0 = run until Ctrl-C")
    args = parser.parse_args()

    # Serial
    try:
        pico = PicoLink(args.port, args.baud, SERIAL_TIMEOUT_S)
    except serial.SerialException as e:
        print(f"[Serial] Could not open {args.port}: {e}")
        sys.exit(1)

    # Camera
    try:
        cap = open_camera()
    except Exception as e:
        print(f"[Camera] {e}")
        cap = None  # allow running without camera; you just won't get descriptions

    print("[Run] Starting autonomous loop. Ctrl-C to exit.")
    t0 = time.time()
    try:
        while True:
            if args.run_seconds and (time.time() - t0) > args.run_seconds:
                break

            # Randomly decide to stop and describe
            if cap is not None and random.random() < RANDOM_STOP_PROB:
                look_and_describe(pico, cap)

            # Check distance
            d = pico.dist_cm()
            if d is not None:
                print(f"[Sensor] Distance: {d:.1f} cm")
            else:
                print("[Sensor] Distance: N/A")

            # If too close, back up a bit and rotate
            if d is not None and d < args.obstacle_cm:
                pico.stop()
                pico.back_burst(0.5)
                random_rotate(pico)
                continue

            # Drive a short “step”
            pico.forward_burst(STEP_DRIVE_S * random.uniform(0.8, 1.3))

            # Sometimes jitter the heading a little
            if random.random() < 0.25:
                random_rotate(pico)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[Run] Interrupted by user.")
    finally:
        try:
            pico.stop()
        except Exception:
            pass
        if cap is not None:
            cap.release()
        print("[Run] Done.")

if __name__ == "__main__":
    main()
