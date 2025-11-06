#!/usr/bin/env python3
"""
rover.py (Autonomous + Vision TTS with CSI fallback)

- Drives in bursts, avoids obstacles via Pico "DIST?"
- Every 20s (default):
    STOP -> SERVO SWEEP -> CAPTURE -> OpenAI Vision -> Google TTS -> PLAY AUDIO
- Camera path:
    1) Try OpenCV VideoCapture (CSI/USB)
    2) If that fails, fall back to gst-launch CSI snapshot (like your working visualText.py)
- Audio:
    Generates MP3 and WAV; tries several players; deletes played file if configured

Prereqs:
    pip install pyserial opencv-python openai google-cloud-texttospeech
    sudo apt install mpg123 ffmpeg sox alsa-utils pulseaudio-utils

Auth env:
    export OPENAI_API_KEY="sk-..."
    export GOOGLE_APPLICATION_CREDENTIALS="/home/tin/gcloud/tts-sa.json"
"""

import os
import sys
import time
import base64
import random
import argparse
import subprocess
import shutil
import re
from typing import Optional, Tuple

import serial
import cv2
from openai import OpenAI
from google.cloud import texttospeech as gtts

# ======= ENV CHECKS =======
OPENAI_KEY = os.environ["OPENAI_API_KEY"] = "Enter Key"
GCREDS     = os.environ.setdefault("GOOGLE_APPLICATION_CREDENTIALS", "/home/tin/gcloud/tts-sa.json")

# ======= BASE CONFIG =======
SERIAL_PORT       = "/dev/ttyACM0"
BAUD              = 115200
SERIAL_TIMEOUT_S  = 0.5

SIDEWAYS_MODE     = True   # STRAFE_R acts like forward
DEFAULT_OBS_CM    = 35.0

STEP_DRIVE_S_DEF  = 1.2
STEP_ROTATE_S_DEF = 0.6

# Camera (OpenCV path)
USE_CSI_CAMERA_DEF = True
CSI_SENSOR_ID      = 0
CAPTURE_W, CAPTURE_H = 1280, 720
FLIP_METHOD        = 6  # rotate 180 (common on Jetson)

# Vision
OPENAI_MODEL  = "gpt-4o-mini"
VISION_PROMPT_DEF = "Describe the scene succinctly. Note key objects, obstacles, and any navigational hints."

# Voice
VOICE_LANG = "en-US"
VOICE_NAME = "en-US-Standard-F"
SPEAK_RATE = 1.0
PITCH      = 0.0
GAIN_DB    = 0.0

# Vision cadence (defaults per request)
RANDOM_STOP_PROB_DEF = 0.0   # no randomness
VISION_EVERY_S_DEF   = 60.0 # speak every 20 seconds

# Robustness
NA_CONSEC_LIMIT   = 3
DIST_PERIOD_S     = 0.12
SMOOTH_N          = 3
AVOID_WINDOW_S    = 6.0
AVOID_MAX         = 5

# Servo sweep BEFORE capture/speaking
SERVO_CHANNEL         = 15
SERVO_SWEEP_ANGLES    = [90, 45,  90]    # [60, 90, 120, 90]
SERVO_SWEEP_DELAY_S   = 1  #0.30

# Audio file handling
DELETE_AFTER_PLAY     = True
KEEP_FAILED_FILES     = True

# ======= COMMAND MAP =======
COMMANDS = {
    "FORWARD":  "STRAFE_R" if SIDEWAYS_MODE else "FWD",
    "BACK":     "STRAFE_L" if SIDEWAYS_MODE else "BACK",
    "ROT_L":    "ROT_L",
    "ROT_R":    "ROT_R",
    "STOP":     "STOP",
    "DIST?":    "DIST?",
    "SERVO":    "SERVO",
}

# ======= CAMERA HELPERS =======
def gstreamer_csi_pipeline(sensor_id=0, capture_w=1280, capture_h=720,
                           display_w=None, display_h=None, framerate=30,
                           flip_method=0):
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

def open_camera(use_csi: bool):
    if use_csi:
        cap = cv2.VideoCapture(
            gstreamer_csi_pipeline(CSI_SENSOR_ID, CAPTURE_W, CAPTURE_H, flip_method=FLIP_METHOD),
            cv2.CAP_GSTREAMER
        )
    else:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
    if not cap.isOpened():
        return None
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

# ======= CSI SNAPSHOT (gst-launch fallback) =======
JPEG_QUALITY  = 85
MAX_SIDE      = 1024
GST_TIMEOUT   = 20
SENSOR_IDS    = [0, 1]
FLIPS         = [FLIP_METHOD, 0]
MODES = [
    {"w":1920, "h":1080, "fps":"30/1"},
    {"w":1280, "h":720,  "fps":"30/1"},
]

def _run(cmd, timeout=GST_TIMEOUT):
    return subprocess.run(["bash","-lc",cmd], text=True, capture_output=True, timeout=timeout)

def _has_gst():
    return shutil.which("gst-launch-1.0") is not None

def _resize_longest(img_bgr, max_side=MAX_SIDE):
    h, w = img_bgr.shape[:2]
    longest = max(h, w)
    if longest <= max_side:
        return img_bgr
    scale = max_side / float(longest)
    return cv2.resize(img_bgr, (int(w*scale), int(h*scale)), interpolation=cv2.INTER_AREA)

def csi_snapshot() -> Tuple[Optional[str], Optional[str]]:
    """Return (jpg_path, meta) or (None, reason) if it fails."""
    if not _has_gst():
        return None, "gst-launch-1.0 not found in PATH."
    tries = []
    ts = int(time.time())
    for sid in SENSOR_IDS:
        for flip in FLIPS:
            for m in MODES:
                out_path = f"/tmp/csi_{sid}_{m['w']}x{m['h']}_{m['fps'].replace('/','-')}_f{flip}_{ts}.jpg"
                cmd = (
                    f"gst-launch-1.0 -q "
                    f"nvarguscamerasrc sensor-id={sid} num-buffers=1 ! "
                    f"'video/x-raw(memory:NVMM),width={m['w']},height={m['h']},framerate={m['fps']}' ! "
                    f"nvvidconv flip-method={flip} ! "
                    f"nvjpegenc quality={JPEG_QUALITY} ! "
                    f"filesink location={out_path} sync=false -e"
                )
                try:
                    p = _run(cmd, timeout=GST_TIMEOUT)
                except subprocess.TimeoutExpired:
                    tries.append(f"sid={sid} {m['w']}x{m['h']}@{m['fps']} f{flip}: TIMEOUT")
                    continue
                ok = (p.returncode == 0 and os.path.exists(out_path) and os.path.getsize(out_path) > 0)
                if ok:
                    return out_path, f"sensor-id={sid}, {m['w']}x{m['h']}@{m['fps']}, flip={flip}"
                tries.append(f"sid={sid} {m['w']}x{m['h']}@{m['fps']} f{flip}: rc={p.returncode} err={(p.stderr or p.stdout)[:160]}")
    return None, "All CSI attempts failed:\n- " + "\n- ".join(tries)

# ======= VISION (OpenAI) =======
_openai_client = OpenAI()

def openai_describe_b64jpeg(b64jpg: str, prompt: str, model: str) -> str:
    # IMPORTANT: use "text" and "image_url" (NOT "input_text")
    msg = _openai_client.chat.completions.create(
        model=model,
        messages=[{
            "role": "user",
            "content": [
                {"type":"text","text": prompt},
                {"type":"image_url","image_url":{"url": f"data:image/jpeg;base64,{b64jpg}"}}
            ]
        }],
        temperature=0.2,
        max_tokens=150
    )
    return msg.choices[0].message.content.strip()

# ======= TTS + PLAYBACK =======
def synth_gcloud_tts_both(text: str, base_name: str) -> Tuple[Optional[str], Optional[str]]:
    client = gtts.TextToSpeechClient()
    input_text = gtts.SynthesisInput(text=text)
    voice = gtts.VoiceSelectionParams(language_code=VOICE_LANG, name=VOICE_NAME)

    mp3_path = f"{base_name}.mp3"
    wav_path = f"{base_name}.wav"

    # MP3
    try:
        cfg_mp3 = gtts.AudioConfig(
            audio_encoding=gtts.AudioEncoding.MP3,
            speaking_rate=SPEAK_RATE, pitch=PITCH, volume_gain_db=GAIN_DB,
        )
        r = client.synthesize_speech(input=input_text, voice=voice, audio_config=cfg_mp3)
        open(mp3_path,"wb").write(r.audio_content)
        print(f"[TTS] MP3: {mp3_path} ({os.path.getsize(mp3_path)} bytes)")
    except Exception as e:
        print("[TTS] MP3 synth failed:", e); mp3_path = None

    # WAV
    try:
        cfg_wav = gtts.AudioConfig(
            audio_encoding=gtts.AudioEncoding.LINEAR16,
            speaking_rate=SPEAK_RATE, pitch=PITCH, volume_gain_db=GAIN_DB,
        )
        r = client.synthesize_speech(input=input_text, voice=voice, audio_config=cfg_wav)
        open(wav_path,"wb").write(r.audio_content)
        print(f"[TTS] WAV: {wav_path} ({os.path.getsize(wav_path)} bytes)")
    except Exception as e:
        print("[TTS] WAV synth failed:", e); wav_path = None

    return mp3_path, wav_path

def try_play_any(mp3_path: Optional[str], wav_path: Optional[str]) -> Tuple[bool, str, Optional[str]]:
    if mp3_path and os.path.exists(mp3_path) and os.path.getsize(mp3_path) > 0:
        mp3_players = [
            ("mpg123", f"mpg123 -q '{mp3_path}'"),
            ("ffplay", f"ffplay -autoexit -nodisp -loglevel error '{mp3_path}'"),
            ("play",   f"play -q '{mp3_path}'"),
            ("paplay", f"paplay '{mp3_path}'"),
        ]
        for exe, cmd in mp3_players:
            if shutil.which(exe):
                print(f"[Audio] Trying {exe} on MP3…")
                rc = subprocess.call(["bash","-lc",cmd])
                if rc == 0: return True, f"Played MP3 with {exe}", mp3_path
                print(f"[Audio] {exe} rc={rc} (MP3)")

    if wav_path and os.path.exists(wav_path) and os.path.getsize(wav_path) > 0:
        wav_players = [
            ("aplay",  f"aplay '{wav_path}'"),
            ("paplay", f"paplay '{wav_path}'"),
            ("ffplay", f"ffplay -autoexit -nodisp -loglevel error '{wav_path}'"),
            ("play",   f"play -q '{wav_path}'"),
        ]
        for exe, cmd in wav_players:
            if shutil.which(exe):
                print(f"[Audio] Trying {exe} on WAV…")
                rc = subprocess.call(["bash","-lc",cmd])
                if rc == 0: return True, f"Played WAV with {exe}", wav_path
                print(f"[Audio] {exe} rc={rc} (WAV)")

    return False, "No working audio player found.", None

# ======= PICO SERIAL =======
_NUM_CM_RE = re.compile(r'([0-9]+(?:\.[0-9]+)?)\s*cm', re.IGNORECASE)

class PicoLink:
    def __init__(self, port: str, baud: int, timeout_s: float):
        self.port = port
        self.baud = baud
        self.timeout = timeout_s
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self.ser.reset_input_buffer()
        print(f"[Serial] Connected on {self.port} @ {self.baud}")

    def _reconnect(self):
        print("[Serial] Reconnecting…")
        try: self.ser.close()
        except Exception: pass
        time.sleep(0.2)
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self.ser.reset_input_buffer()

    def _drain_input(self, quiet_ms=150):
        last = time.time(); quiet_s = quiet_ms/1000.0
        while True:
            try: n = self.ser.in_waiting
            except (serial.SerialException, OSError):
                self._reconnect(); continue
            if n > 0:
                _ = self.ser.read(n); last = time.time()
            else:
                if time.time() - last >= quiet_s: break
                time.sleep(0.01)

    def _read_until_number_cm(self, window_s=1.2, max_lines=400):
        end = time.time() + window_s; read = 0
        while time.time() < end and read < max_lines:
            try: line = self.ser.readline()
            except (serial.SerialException, OSError):
                self._reconnect(); continue
            if not line: time.sleep(0.005); continue
            s = line.decode(errors="ignore").strip(); read += 1
            if "none" in s.lower(): return None
            m = _NUM_CM_RE.search(s)
            if m:
                try:
                    val = float(m.group(1))
                    if 0.5 <= val <= 500.0: return val
                except ValueError:
                    pass
        return None

    def send(self, cmd: str) -> list[str]:
        try:
            self.ser.write((cmd.strip()+"\n").encode()); time.sleep(0.05)
            lines = []; t_end = time.time() + 0.25
            while time.time() < t_end and len(lines) < 8:
                line = self.ser.readline()
                if not line: break
                s = line.decode(errors="ignore").strip()
                if s: lines.append(s)
            return lines
        except (serial.SerialException, OSError):
            self._reconnect(); return []

    def dist_cm(self):
        self._drain_input(quiet_ms=150)
        try: self.ser.write(b"DIST?\n")
        except (serial.SerialException, OSError):
            self._reconnect(); self.ser.write(b"DIST?\n")
        return self._read_until_number_cm(window_s=1.2, max_lines=400)

    def stop(self):
        try: self.ser.write(b"STOP\n")
        except (serial.SerialException, OSError):
            self._reconnect(); self.ser.write(b"STOP\n")
        time.sleep(0.02)
        try:
            n = self.ser.in_waiting
            if n: _ = self.ser.read(n)
        except (serial.SerialException, OSError):
            self._reconnect()

    def _burst(self, cmd: str, secs: float, period: float = 0.05):
        t_end = time.time() + secs
        while time.time() < t_end:
            try: self.ser.write((cmd+"\n").encode())
            except (serial.SerialException, OSError):
                self._reconnect(); self.ser.write((cmd+"\n").encode())
            time.sleep(0.01)
            try:
                n = self.ser.in_waiting
                if n: _ = self.ser.read(n)
            except (serial.SerialException, OSError):
                self._reconnect()
            sleep_left = period - 0.01
            if sleep_left > 0: time.sleep(sleep_left)

    def forward_burst(self, secs: float): self._burst(COMMANDS["FORWARD"], secs)
    def back_burst(self, secs: float):    self._burst(COMMANDS["BACK"], secs)
    def rotate_right_burst(self, secs: float): self._burst(COMMANDS["ROT_R"], secs)
    def rotate_left_burst(self, secs: float):  self._burst(COMMANDS["ROT_L"], secs)

    def servo(self, ch: int, angle_deg: float):
        try: self.ser.write((f"{COMMANDS['SERVO']} {ch} {angle_deg:.1f}\n").encode())
        except (serial.SerialException, OSError):
            self._reconnect(); self.ser.write((f"{COMMANDS['SERVO']} {ch} {angle_deg:.1f}\n").encode())
        time.sleep(0.02)
        try:
            n = self.ser.in_waiting
            if n: _ = self.ser.read(n)
        except (serial.SerialException, OSError):
            self._reconnect()

# ======= DESCRIBE + SPEAK =======
def _describe_text_from_bgr(img_bgr, prompt: str) -> Optional[str]:
    img_bgr = _resize_longest(img_bgr, MAX_SIDE)
    ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
    if not ok:
        print("[Vision] JPEG encode failed."); return None
    b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
    try:
        return openai_describe_b64jpeg(b64, prompt, OPENAI_MODEL)
    except Exception as e:
        print("[Vision] OpenAI error:", e)
        return None

def describe_and_speak(cap: Optional[cv2.VideoCapture], prompt: str):
    """
    Try OpenCV frame first; if cap is None or read fails, use CSI gst snapshot.
    """
    img_bgr = None
    method = None

    if cap is not None:
        ok, frame = cap.read()
        if ok:
            img_bgr = frame
            method = "OpenCV"
        else:
            print("[Camera] OpenCV read failed; falling back to CSI snapshot.")

    if img_bgr is None:
        path, meta = csi_snapshot()
        if path is None:
            print("[Camera] CSI snapshot failed:", meta)
            return
        img_bgr = cv2.imread(path)
        if img_bgr is None:
            print("[Camera] OpenCV failed to read CSI file:", path)
            return
        method = f"CSI snapshot ({meta})"

    desc = _describe_text_from_bgr(img_bgr, prompt)
    if not desc:
        return
    print(f"\n[Scene Description via {method}]\n{desc}\n")

    base = f"speech_{int(time.time())}"
    mp3_path, wav_path = synth_gcloud_tts_both(desc, base)
    ok, msg, played = try_play_any(mp3_path, wav_path)
    print("Audio:", msg)

    if ok and DELETE_AFTER_PLAY and played:
        try: os.remove(played)
        except Exception: pass
        other = mp3_path if played == wav_path else wav_path
        if other and not KEEP_FAILED_FILES:
            try: os.remove(other)
            except Exception: pass
    else:
        print("[Audio] Kept files for debugging.")

# ======= HIGH-LEVEL =======
def random_rotate(pico: PicoLink, step_rotate_s: float):
    if random.random() < 0.5:
        pico.rotate_left_burst(step_rotate_s * random.uniform(0.8, 1.6))
    else:
        pico.rotate_right_burst(step_rotate_s * random.uniform(0.8, 1.6))

def look_pan_describe_speak(pico: PicoLink, cap: Optional[cv2.VideoCapture], prompt: str):
    pico.stop()
    time.sleep(0.15)
    if SERVO_CHANNEL is not None and SERVO_SWEEP_ANGLES:
        for ang in SERVO_SWEEP_ANGLES:
            pico.servo(SERVO_CHANNEL, ang)
            time.sleep(SERVO_SWEEP_DELAY_S)
    describe_and_speak(cap, prompt)

# ======= MAIN =======
def main():
    parser = argparse.ArgumentParser(description="Autonomous wander + servo pan + describe + speak")
    parser.add_argument("--port", "-p", default=SERIAL_PORT)
    parser.add_argument("--baud", "-b", type=int, default=BAUD)
    parser.add_argument("--obstacle_cm", type=float, default=DEFAULT_OBS_CM)
    parser.add_argument("--run_seconds", type=int, default=0, help="0 = run until Ctrl-C")
    parser.add_argument("--usb-cam", action="store_true", help="Use /dev/video0 instead of CSI.")
    parser.add_argument("--safe", action="store_true", help="Lower speed/shorter steps & higher caution.")
    parser.add_argument("--vision-interval", type=float, default=VISION_EVERY_S_DEF,
                        help="Describe every N seconds (<=0 disables interval mode).")
    parser.add_argument("--random-vision-prob", type=float, default=RANDOM_STOP_PROB_DEF,
                        help="Chance per loop to describe when interval mode is disabled.")
    parser.add_argument("--prompt", type=str, default=VISION_PROMPT_DEF, help="Custom prompt for description.")
    args = parser.parse_args()

    # Locals (avoid Python 'global' quirks)
    obstacle_cm  = args.obstacle_cm
    step_drive_s = STEP_DRIVE_S_DEF
    step_rot_s   = STEP_ROTATE_S_DEF
    use_csi      = not args.usb_cam
    prompt       = args.prompt

    if args.safe:
        step_drive_s = 0.7
        step_rot_s   = 0.45
        obstacle_cm  = max(obstacle_cm, 60.0)

    # Serial
    try:
        pico = PicoLink(args.port, args.baud, SERIAL_TIMEOUT_S)
    except serial.SerialException as e:
        print(f"[Serial] Could not open {args.port}: {e}")
        sys.exit(1)

    # Camera try (OpenCV); if it fails, we keep cap=None and use CSI snapshot later
    cap = open_camera(use_csi)
    if cap is None:
        print("[Camera] Could not open camera stream; will use CSI snapshot fallback.")

    # Vision cadence
    use_interval  = args.vision_interval and args.vision_interval > 0.0
    vision_every  = max(0.0, args.vision_interval or 0.0)
    random_prob   = max(0.0, min(1.0, args.random_vision_prob))

    print("[Run] Starting autonomous loop. Ctrl-C to exit.")
    t0 = time.time()
    last_vision = 0.0
    last_dist_poll = 0.0
    dist_buf = []
    na_streak = 0
    avoid_hits = []

    try:
        while True:
            now = time.time()
            if args.run_seconds and (now - t0) > args.run_seconds:
                break

            # Vision cadence
            do_vision = False
            if use_interval and (now - last_vision) >= vision_every:
                do_vision = True
            elif (not use_interval) and (random.random() < random_prob):
                do_vision = True

            if do_vision:
                look_pan_describe_speak(pico, cap, prompt)
                last_vision = now

            # Distance read (rate-limited + smoothing)
            d = None
            if (now - last_dist_poll) >= DIST_PERIOD_S:
                last_dist_poll = now
                rd = pico.dist_cm()
                if rd is not None:
                    na_streak = 0
                    dist_buf.append(rd)
                    if len(dist_buf) > SMOOTH_N:
                        dist_buf.pop(0)
                    d = sum(dist_buf) / len(dist_buf)
                    print(f"[Sensor] Distance: {d:.1f} cm (n={len(dist_buf)})")
                else:
                    na_streak += 1
                    print(f"[Sensor] Distance: N/A (streak={na_streak})")

            # Avoidance decision
            too_close = (d is not None and d < obstacle_cm) or (na_streak >= NA_CONSEC_LIMIT)
            if too_close:
                pico.stop()
                pico.back_burst(0.5)

                avoid_hits = [t for t in avoid_hits if now - t < AVOID_WINDOW_S]
                avoid_hits.append(now)

                if len(avoid_hits) >= AVOID_MAX:
                    print("[Recover] Too many avoids: backing longer + bigger turn")
                    pico.back_burst(1.2)
                    if random.random() < 0.5:
                        pico.rotate_left_burst(step_rot_s * 2.2)
                    else:
                        pico.rotate_right_burst(step_rot_s * 2.2)
                    avoid_hits.clear()
                else:
                    random_rotate(pico, step_rot_s)
                time.sleep(0.05)
                continue

            # Drive a short “step”
            pico.forward_burst(step_drive_s * random.uniform(0.8, 1.3))

            # Sometimes jitter heading
            if random.random() < 0.25:
                if random.random() < 0.5:
                    pico.rotate_left_burst(step_rot_s * 0.7)
                else:
                    pico.rotate_right_burst(step_rot_s * 0.7)

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

