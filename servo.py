# servo.py
# PCA9685 servo driver glue for main.py
# - init():   sets up I2C + PCA9685 and centers all channels
# - set_servo(ch, angle_deg): set target angle on a channel (0..180)
# Optional helpers: center_all, park, sweep, set_trim, set_range

import time
from machine import I2C, Pin
from pca9685 import PCA9685

# ====== CONFIGURE YOUR PINS HERE ======
# Examples:
# - I2C0 on GP0/GP1:  I2C_ID=0, SDA_PIN=0,  SCL_PIN=1
# - I2C0 on GP20/21: I2C_ID=0, SDA_PIN=20, SCL_PIN=21   <-- common on Pico projects
# - I2C1 on GP2/3:   I2C_ID=1, SDA_PIN=2,  SCL_PIN=3
I2C_ID   = 0
SDA_PIN  = 20
SCL_PIN  = 21
I2C_FREQ = 400_000
ADDR     = 0x40

# Servo timing (tune to your servo; widen if endpoints buzz)
SERVO_HZ     = 50
SERVO_MIN_US = 1000     # ~0°
SERVO_MAX_US = 2000     # ~180°

# Per-channel trims in degrees (fine center adjust)
_TRIMS = [0.0] * 16

# Internal: PCA9685 instance
_p = None

# ---------- Utilities from your test harness ----------
def _counts_from_us(pulse_us, freq=SERVO_HZ):
    period_us = 1_000_000.0 / freq    # e.g., 20,000 us at 50 Hz
    # 12-bit range is 0..4095
    return int(min(4095, max(0, round(4096.0 * pulse_us / period_us))))

def _angle_to_counts(angle_deg):
    # Clamp and convert to pulse width then counts
    a = max(0.0, min(180.0, float(angle_deg)))
    pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (a / 180.0)
    return _counts_from_us(pulse, SERVO_HZ)

def _i2c_scan_or_die(i2c):
    found = i2c.scan()
    print("I2C scan:", [hex(x) for x in found])
    if ADDR not in found:
        raise RuntimeError("PCA9685 not found at 0x%02X. Check wiring, power, and pins." % ADDR)

# ---------- Public API used by main.py ----------
def init(i2c_id=I2C_ID, sda=SDA_PIN, scl=SCL_PIN, addr=ADDR, i2c_freq=I2C_FREQ, servo_hz=SERVO_HZ):
    """Initialize I2C + PCA9685 and center all channels."""
    global _p, SERVO_HZ
    SERVO_HZ = int(servo_hz)

    print("Servo init: I2C id=%d SDA=GP%d SCL=GP%d @ %d Hz" % (i2c_id, sda, scl, i2c_freq))
    i2c = I2C(i2c_id, sda=Pin(sda), scl=Pin(scl), freq=i2c_freq)
    _i2c_scan_or_die(i2c)

    _p = PCA9685(i2c, address=addr)
    _p.freq(SERVO_HZ)
    print("PCA9685 @ 0x%02X ready, freq=%d Hz" % (addr, _p.freq()))

    center_all(angle=90)
    print("Servo: all channels centered (90°)")

def set_servo(ch, angle_deg):
    """Set servo on channel ch (0..15) to angle_deg (0..180)."""
    if _p is None:
        raise RuntimeError("servo.init() must be called before set_servo()")
    ch = int(ch)
    if not (0 <= ch <= 15):
        raise ValueError("Channel out of range (0..15)")
    a = float(angle_deg) + float(_TRIMS[ch])
    cnt = _angle_to_counts(a)
    _p.duty(ch, cnt)

# ---------- Nice-to-have helpers ----------
def center_all(angle=90):
    """Center every channel to the given angle."""
    if _p is None:
        return
    val = _angle_to_counts(angle)
    for ch in range(16):
        _p.duty(ch, val)

def park(ch=None, angle=90):
    """Park either one channel or all at angle."""
    if _p is None:
        return
    if ch is None:
        center_all(angle)
    else:
        ch = int(ch)
        _p.duty(ch, _angle_to_counts(angle))

def sweep(ch=0, step=2, delay=0.02):
    """Manual sweep test (Ctrl+C to stop)."""
    if _p is None:
        raise RuntimeError("servo.init() first")
    print("Servo sweep on CH%d" % ch)
    while True:
        for a in range(0, 181, step):
            set_servo(ch, a)
            time.sleep(delay)
        for a in range(180, -1, -step):
            set_servo(ch, a)
            time.sleep(delay)

def set_trim(ch, trim_deg=0.0):
    """Apply small per-channel trim in degrees (e.g., -5..+5)."""
    ch = int(ch)
    _TRIMS[ch] = float(trim_deg)

def set_range(min_us=None, max_us=None):
    """Optionally adjust global servo pulse range."""
    global SERVO_MIN_US, SERVO_MAX_US
    if min_us is not None:
        SERVO_MIN_US = int(min_us)
    if max_us is not None:
        SERVO_MAX_US = int(max_us)
