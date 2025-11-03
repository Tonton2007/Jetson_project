from machine import I2C, Pin
from pca9685 import PCA9685
import time

i2c = I2C(1, sda=Pin(26), scl=Pin(27), freq=400000)
p = PCA9685(i2c)
p.freq(50)

def angle(a):
    SERVO_MIN_US, SERVO_MAX_US = 1000, 2000
    period_us = 1_000_000/50
    pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US)*(max(0,min(180,a))/180)
    p.duty(0, int(4096*pulse/period_us))

angle(0);   time.sleep(1)
angle(90);  time.sleep(1)
angle(180); time.sleep(1)
angle(90)