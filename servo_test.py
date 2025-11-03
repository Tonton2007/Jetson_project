# servo_test.py
# Quick test harness for PCA9685 + hobby servo on Raspberry Pi Pico

import time
from machine import Pin, I2C
from pca9685 import PCA9685

# ====== CONFIGURE YOUR PINS HERE ======
# Default Pico I2C0 pins: SDA=GP0, SCL=GP1
# If you wired like your earlier message (SDA=GP1, SCL=GP2), change below:
I2C_ID   = 1
SDA_PIN  = 26   # set to 1 if you used GP1 for SDA
SCL_PIN  = 27   # set to 2 if you used GP2 for SCL
I2C_FREQ = 400_000
ADDR     = 0x40

# Servo timing (tune to your servo if needed)
SERVO_HZ     = 50
SERVO_MIN_US = 1000   # ~0°
SERVO_MAX_US = 2000   # ~180°

def counts_from_us(pulse_us, freq=SERVO_HZ):
    period_us = 1_000_000.0 / freq    # e.g., 20,000us at 50Hz
    return int(min(4095, max(0, round(4096.0 * pulse_us / period_us))))

def angle_to_counts(angle_deg):
    angle = max(0, min(180, angle_deg))
    pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (angle / 180.0)
    return counts_from_us(pulse)

def i2c_scan_or_die(i2c):
    found = i2c.scan()
    print("I2C scan:", [hex(x) for x in found])
    if ADDR not in found:
        raise RuntimeError("PCA9685 not found at 0x%02X. Check wiring, power, and pins." % ADDR)

def center_all(p, angle=90):
    val = angle_to_counts(angle)
    for ch in range(16):
        p.duty(ch, val)

def sweep_one(p, ch=0, step=2, delay=0.02):
    print("Sweeping channel", ch)
    while True:
        for a in range(0, 181, step):
            p.duty(ch, angle_to_counts(a))
            time.sleep(delay)
        for a in range(180, -1, -step):
            p.duty(ch, angle_to_counts(a))
            time.sleep(delay)

def main():
    print("Setting up I2C on id=%d SDA=GP%d SCL=GP%d" % (I2C_ID, SDA_PIN, SCL_PIN))
    i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)

    i2c_scan_or_die(i2c)

    p = PCA9685(i2c, address=ADDR)
    p.freq(SERVO_HZ)   # 50 Hz for servos
    print("PCA9685 @ 0x%02X ready, freq=%d Hz" % (ADDR, p.freq()))

    # put everything safe/centered
    center_all(p, angle=90)
    print("All channels centered (90°). Starting sweep demo on CH0 in 2s...")
    time.sleep(2)

    # Sweep the servo connected to CH0
    sweep_one(p, ch=0)

if __name__ == "__main__":
    main()
