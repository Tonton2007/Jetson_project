from machine import Pin, PWM
import time  

# Motor pins
In1 = Pin(6, Pin.OUT)
In2 = Pin(7, Pin.OUT)
EN_A = PWM(Pin(8))   # <-- PWM instead of Pin.OUT

#In3 = Pin(4, Pin.OUT)
#In4 = Pin(3, Pin.OUT)
#EN_B = PWM(Pin(2))   # <-- PWM instead of Pin.OUT

# Set PWM frequency (1 kHz is common for motor control)
EN_A.freq(1000)
#EN_B.freq(1000)

# A function to set motor speed (0-65535)
def set_speed(percent):
    duty = int((percent/100) * 65535)   # convert % to 16-bit duty cycle
    EN_A.duty_u16(duty)
#    EN_B.duty_u16(duty)

# Forward
def move_forward():
    In1.high(); In2.low()
 #   In3.high(); In4.low()

# Backward
def move_backward():
    In1.low(); In2.high()
  #  In3.low(); In4.high()

# Turn Right
def turn_right():
    In1.low(); In2.low()
   # In3.low(); In4.high()

# Turn Left
def turn_left():
    In1.low(); In2.high()
    #In3.low(); In4.low()

# Stop
def stop():
    In1.low(); In2.low()
  #  In3.low(); In4.low()
    set_speed(0)

# ======= TEST LOOP =======
while True:
    set_speed(40)              # 40% speed
    move_forward()
    print("Forward at 40%")
    time.sleep(2)

    stop()
    print("Stop")
    time.sleep(1)

    set_speed(50)              # 50% speed
    move_backward()
    print("Backward at 50%")
    time.sleep(2)

    stop()
    print("Stop")
    time.sleep(1)
    
    set_speed(100)              # 100% speed
    move_forward()
    print("Forward at 100%")
    time.sleep(2)

    stop()
    print("Stop")
    time.sleep(1)
