# ultra.py
# HC-SR04 style ultrasonic distance sensor
# TRIG = GP20 (output)
# ECHO = GP21 (input)
#
# NOTE: Make sure ECHO is 3.3V safe before wiring directly to Pico.

import machine, time

TRIG_PIN = 26
ECHO_PIN = 27

_trig = machine.Pin(TRIG_PIN, machine.Pin.OUT)
_echo = machine.Pin(ECHO_PIN, machine.Pin.IN)

# speed of sound ~343 m/s = 0.0343 cm/us
# distance (cm) = (echo_time_us * 0.0343) / 2

def read_cm(timeout_us=30_000):
    """
    Returns distance in cm (float), or None if timeout.
    timeout_us is max time waiting for echo.
    """
    # ensure trig low
    _trig.value(0)
    time.sleep_us(2)

    # 10us high pulse on trig
    _trig.value(1)
    time.sleep_us(10)
    _trig.value(0)

    # measure echo high pulse
    dur = machine.time_pulse_us(_echo, 1, timeout_us)
    if dur < 0:
        # timeout or invalid
        return None

    dist_cm = (dur * 0.0343) / 2.0
    return dist_cm
