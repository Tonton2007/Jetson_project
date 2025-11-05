# main.py
# High-level robot command loop.
#
# Jetson (or you over USB) sends text commands like:
#   DRIVE 0 1 0
#   STOP
#   SPEED 30
#   PWM_ON
#   PWM_OFF
#   BRAKE
#   COAST
#   SERVO 0 45
#   DIST?
#
# Pico responds on both USB CDC and UART0.

import sys, time
import machine
import uselect

import servo
import ultra
import drive

# ---------- LED blink on RX ----------
try:
    LED = machine.Pin("LED", machine.Pin.OUT)
except:
    LED = machine.Pin(25, machine.Pin.OUT)

def led_pulse(ms=40):
    try:
        LED.value(1)
        time.sleep_ms(ms)
        LED.value(0)
    except:
        pass

# ---------- IO: UART0 and USB CDC ----------
uart = machine.UART(
    0,
    baudrate=115200,
    tx=machine.Pin(0),
    rx=machine.Pin(1),
)
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

# ---------- INIT SUBSYSTEMS ----------
drive.init()
servo.init()

def println_all(msg):
    # send feedback to USB and UART0
    try:
        print(msg)
    except:
        pass
    try:
        uart.write((msg+"\n").encode())
    except:
        pass

def handle_line(line_raw):
    line = line_raw.strip()
    if not line:
        return
    upper = line.upper()

    # ----- Motion / drive -----
    if upper == "FWD":
        drive.drive_mecanum(+1, 0, 0)
        println_all("OK: FWD")
    elif upper == "BACK":
        drive.drive_mecanum(-1, 0, 0)
        println_all("OK: BACK")
    elif upper == "STRAFE_L":
        drive.drive_mecanum(0, -1, 0)
        println_all("OK: STRAFE_L")
    elif upper == "STRAFE_R":
        drive.drive_mecanum(0, +1, 0)
        println_all("OK: STRAFE_R")
    elif upper == "ROT_L":
        drive.drive_mecanum(0, 0, +1)
        println_all("OK: ROT_L")
    elif upper == "ROT_R":
        drive.drive_mecanum(0, 0, -1)
        println_all("OK: ROT_R")
    elif upper == "STOP":
        drive.stop_all()
        println_all("OK: STOP")

    # ----- Brake / coast -----
    elif upper == "BRAKE":
        drive.brake_mode(True)
        println_all("OK: BRAKE MODE")
    elif upper == "COAST":
        drive.brake_mode(False)
        println_all("OK: COAST MODE")

    # ----- Speed cap -----
    elif upper.startswith("SPEED "):
        # SPEED NN
        parts = line.split()
        if len(parts) == 2:
            try:
                pct = int(parts[1])
                drive.set_speed_cap(pct)
                println_all("OK: SPEED %d" % pct)
            except:
                println_all("ERR: BAD SPEED")
        else:
            println_all("ERR: BAD SPEED")

    # ----- Toggle PWM / digital EN -----
    elif upper == "PWM_ON":
        drive.use_pwm_mode()
        println_all("OK: PWM_ON")
    elif upper == "PWM_OFF":
        drive.use_digital_mode()
        println_all("OK: PWM_OFF")

    # ----- Self test -----
    elif upper == "SELFTEST":
        println_all("Self-test running...")
        drive.self_test(duration_s=1.0)
        println_all("OK: SELFTEST DONE")

    # ----- Direct vector drive -----
    elif upper.startswith("DRIVE "):
        # DRIVE vx vy w
        parts = line.split()
        if len(parts) == 4:
            try:
                vx = float(parts[1])
                vy = float(parts[2])
                w  = float(parts[3])
                drive.drive_mecanum(vx, vy, w)
                println_all("OK: DRIVE %.2f %.2f %.2f" % (vx, vy, w))
            except:
                println_all("ERR: BAD DRIVE")
        else:
            println_all("ERR: BAD DRIVE")

    # ----- Servo control -----
    elif upper.startswith("SERVO "):
        # SERVO ch angle
        parts = line.split()
        if len(parts) == 3:
            try:
                ch = int(parts[1])
                ang = float(parts[2])
                servo.set_servo(ch, ang)
                println_all("OK: SERVO ch%d -> %.1f deg" % (ch, ang))
            except:
                println_all("ERR: BAD SERVO")
        else:
            println_all("ERR: BAD SERVO")

    # ----- Ultrasonic distance -----
    elif upper == "DIST?":
        d = ultra.read_cm()
        if d is None:
            println_all("DIST: NONE")
        else:
            println_all("DIST: %.1f cm" % d)

    # ----- Telemetry -----
    elif upper == "STATUS?":
        st = drive.status()
        println_all("STATUS: %s" % st)

    # ----- Echo test -----
    elif upper.startswith("ECHO "):
        msg = line[5:].lstrip()
        println_all(msg)

    else:
        println_all("ERR: UNKNOWN")

    led_pulse()

# ---------- Main loop ----------
println_all("Pico main ready. (PWM wheels + servo + ultrasonic)")

buf_uart = b""

try:
    while True:
        # USB CDC commands
        if poll.poll(0):
            cmd = sys.stdin.readline()
            if cmd:
                handle_line(cmd)

        # UART0 commands
        if uart.any():
            ch = uart.read(1)
            if ch:
                if ch == b'\n':
                    try:
                        handle_line(buf_uart.decode())
                    except:
                        println_all("ERR: DECODE")
                    buf_uart = b""
                elif ch != b'\r':
                    buf_uart += ch

        time.sleep(0.002)

except KeyboardInterrupt:
    pass
finally:
    drive.stop_all()
    println_all("STOP")
