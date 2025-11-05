# drive.py
# Raspberry Pi Pico (MicroPython)
# 4x DC motors via 2x L298N, PWM on EN pins for speed control.
# Supports mecanum drive: vx (strafe), vy (forward), w (rotate).

import machine, time

# ---------- CONFIG ----------
PWM_FREQ   = 1000      # 1 kHz works well for L298N
DEADBAND   = 0.04      # below this magnitude, treat as 0
CLIP       = 1.0       # final clip after normalization
SPEED_CAP  = 1.0       # global speed cap (0.0..1.0), change via set_speed_cap(%)

# ---------- PIN MAP ----------
# Each motor has IN1, IN2 (direction), EN (PWM speed)
# Adjust these to your wiring. These reflect your earlier mapping.
MOTORS = {
    # Front-Left
    "FL": {
        "IN1": machine.Pin(5,  machine.Pin.OUT),
        "IN2": machine.Pin(4,  machine.Pin.OUT),
        "ENp": machine.Pin(6,  machine.Pin.OUT)   # will be converted to PWM
    },
    # Rear-Left
    "RL": {
        "IN1": machine.Pin(8,  machine.Pin.OUT),
        "IN2": machine.Pin(7,  machine.Pin.OUT),
        "ENp": machine.Pin(9,  machine.Pin.OUT)
    },
    # Front-Right
    "FR": {
        "IN1": machine.Pin(14, machine.Pin.OUT),
        "IN2": machine.Pin(15, machine.Pin.OUT),
        "ENp": machine.Pin(13, machine.Pin.OUT)
    },
    # Rear-Right
    "RR": {
        "IN1": machine.Pin(17, machine.Pin.OUT),
        "IN2": machine.Pin(18, machine.Pin.OUT),
        "ENp": machine.Pin(16, machine.Pin.OUT)
    },
}

# runtime
_pwms = {}
_brake_mode = False  # False = coast (IN1=0, IN2=0), True = brake (IN1=1, IN2=1)

# ---------- HELPERS ----------
def _u16(x_unit):
    if x_unit <= 0: return 0
    if x_unit >= 1: return 65535
    return int(x_unit * 65535)

def _apply_stop_pins(mpins):
    if _brake_mode:
        mpins["IN1"].value(1)
        mpins["IN2"].value(1)
    else:
        mpins["IN1"].value(0)
        mpins["IN2"].value(0)

def init():
    """Call once after import."""
    global _pwms
    # setup PWM on EN pins
    for name, m in MOTORS.items():
        p = machine.PWM(m["ENp"])
        p.freq(PWM_FREQ)
        p.duty_u16(0)
        _pwms[name] = p
        # ensure direction low initially
        m["IN1"].value(0)
        m["IN2"].value(0)

def set_speed_cap(percent):
    """Set global speed cap 0..100%."""
    global SPEED_CAP
    if percent < 0:   percent = 0
    if percent > 100: percent = 100
    SPEED_CAP = percent / 100.0

def brake_mode(on=True):
    """True=brake, False=coast."""
    global _brake_mode
    _brake_mode = bool(on)
    stop_all()

def coast_mode():
    brake_mode(False)

def stop_motor(name):
    """Stop one motor, obeying brake/coast."""
    mpins = MOTORS[name]
    _apply_stop_pins(mpins)
    _pwms[name].duty_u16(0)

def stop_all():
    for n in MOTORS:
        stop_motor(n)

def _apply_motor(name, v):
    """
    Set motor speed v in -1..+1.
    Sign = direction. Magnitude = throttle (scaled by SPEED_CAP).
    """
    v = max(-1.0, min(1.0, v))
    mag = abs(v)

    mpins = MOTORS[name]
    if mag < DEADBAND:
        # effectively stopped
        _apply_stop_pins(mpins)
        _pwms[name].duty_u16(0)
        return

    # direction
    fwd = (v >= 0.0)
    mpins["IN1"].value(1 if fwd else 0)
    mpins["IN2"].value(0 if fwd else 1)

    # throttle
    duty = mag * SPEED_CAP
    _pwms[name].duty_u16(_u16(duty))

def drive_mecanum(vx, vy, w):
    """
    Mecanum mixer with conventional signs:
      vx: +right / -left       (-1..+1)
      vy: +forward / -back     (-1..+1)
      w : +CCW rotate / -CW    (-1..+1)
    Wheel formulas (standard):
      FL = vy + vx + w
      FR = vy - vx - w
      RL = vy - vx + w
      RR = vy + vx - w
    """
    vx = max(-1, min(1, vx))
    vy = max(-1, min(1, vy))
    w  = max(-1, min(1, w))

    fl = vy + vx + w
    fr = vy - vx - w
    rl = vy - vx + w
    rr = vy + vx - w

    # normalize so max magnitude <= 1
    m = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))
    fl /= m; fr /= m; rl /= m; rr /= m

    # final clip
    fl = max(-CLIP, min(CLIP, fl))
    fr = max(-CLIP, min(CLIP, fr))
    rl = max(-CLIP, min(CLIP, rl))
    rr = max(-CLIP, min(CLIP, rr))

    _apply_motor("FL", fl)
    _apply_motor("FR", fr)
    _apply_motor("RL", rl)
    _apply_motor("RR", rr)

def self_test(step_s=1.0):
    """
    Quick go/no-go: fwd, rev, stop (ignores speed cap by just commanding 1.0).
    """
    # forward
    for n in MOTORS:
        _apply_motor(n, +1.0)
    time.sleep(step_s)

    # reverse
    for n in MOTORS:
        _apply_motor(n, -1.0)
    time.sleep(step_s)

    stop_all()

def status():
    return {
        "pwm_freq": PWM_FREQ,
        "deadband": DEADBAND,
        "clip": CLIP,
        "speed_cap_pct": int(SPEED_CAP * 100),
        "brake_mode": _brake_mode,
    }

# ---------- OPTIONAL DEMO ----------
# This mirrors your single-motor pattern but drives all wheels via mecanum.
# Comment out if you import this from another file.
if __name__ == "__main__":
    init()
    set_speed_cap(100)  # 40% like your example

    # Forward 2s
    drive_mecanum(vx=0, vy=+1, w=0)
    print("Forward at 40%")
    time.sleep(2)

    stop_all()
    print("Stop")
    time.sleep(1)

    # Backward 2s @ 50%
    set_speed_cap(100)
    drive_mecanum(vx=0, vy=-1, w=0)
    print("Backward at 50%")
    time.sleep(2)

    stop_all()
    print("Stop")
    time.sleep(1)

    # Forward 2s @ 100%
    set_speed_cap(100)
    drive_mecanum(vx=0, vy=+1, w=0)
    print("Forward at 100%")
    time.sleep(2)

    stop_all()
    print("Stop")
