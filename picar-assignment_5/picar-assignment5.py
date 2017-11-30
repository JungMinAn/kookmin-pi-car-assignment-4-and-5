"""
# Date: 2017/11/06
# file name: picar-assignment5.py
"""

# Import raspberry pi's GPIO module and time module
import RPi.GPIO as GPIO
import time

# Disable warning text
GPIO.setwarnings(False)

# GPIO mode setting
GPIO.setmode(GPIO.BOARD)

# Trig, echo pin number
trig = 33
echo = 31

# Trig, echo in/out setting
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)


def REVERSE(x):
    """
    Reverse for forward
    :return: boolean
    """
    if x == True:
        return False
    elif x == False:
        return True


# Forward boolean
forward0 = True
forward1 = False

# Backward define
backward0 = REVERSE(forward0)
backward1 = REVERSE(forward1)

# Left motor pin number
MotorLeft_A = 12
MotorLeft_B = 11
MotorLeft_PWM = 35

# Right motor pin number
MotorRight_A = 15
MotorRight_B = 13
MotorRight_PWM = 37


def leftmotor(x):
    """
    Left motor main define
    """
    if x == True:
        GPIO.output(MotorLeft_A, GPIO.HIGH)
        GPIO.output(MotorLeft_B, GPIO.LOW)
    elif x == False:
        GPIO.output(MotorLeft_A, GPIO.LOW)
        GPIO.output(MotorLeft_B, GPIO.HIGH)
    else:
        print
        'Config Error'


def rightmotor(x):
    """
    Right motor main define
    """
    if x == True:
        GPIO.output(MotorRight_A, GPIO.LOW)
        GPIO.output(MotorRight_B, GPIO.HIGH)
    elif x == False:
        GPIO.output(MotorRight_A, GPIO.HIGH)
        GPIO.output(MotorRight_B, GPIO.LOW)


# Left motor's GPIO out port setting
GPIO.setup(MotorLeft_A, GPIO.OUT)
GPIO.setup(MotorLeft_B, GPIO.OUT)
GPIO.setup(MotorLeft_PWM, GPIO.OUT)

# Right motor's GPIO out port setting
GPIO.setup(MotorRight_A, GPIO.OUT)
GPIO.setup(MotorRight_B, GPIO.OUT)
GPIO.setup(MotorRight_PWM, GPIO.OUT)

# Motor power up setting
LeftPwm = GPIO.PWM(MotorLeft_PWM, 100)
RightPwm = GPIO.PWM(MotorRight_PWM, 100)


def rightSwingTurn(speed1, speed2, running_time):
    """
    Right swing turn main module
    :param speed1: Left speed
    :param speed2: Right speed
    :param running_time: motor running time
    """
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(speed1)
    RightPwm.ChangeDutyCycle(speed2)
    time.sleep(running_time)


def leftSwingTurn(speed1, speed2, running_time):
    """
    Left swing turn main module
    :param speed1: Right speed
    :param speed2: Left speed
    :param running_time: motor running time
    """
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed2)
    RightPwm.ChangeDutyCycle(speed1)
    time.sleep(running_time)


def rightPointTurn(speed, running_time):
    """
    Right point turn main module
    :param speed: motor running speed
    :param running_time: motor running time
    """
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(backward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def rightPointTurn_any(speed):
    """
    Right point turn main module
    :param speed: motor running speed
    """
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(backward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    IO = 1
    while IO == 1:
        LeftPwm.ChangeDutyCycle(speed)
        RightPwm.ChangeDutyCycle(speed)
        time.sleep(0.01)
        c = GPIO.input(centerled)
        IO = c


def leftPointTurn(speed, running_time):
    """
    Left point turn main module
    :param speed: Motor running speed
    :param running_time: Motor running time
    """
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    leftmotor(backward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def leftPointTurn_any(speed):
    """
    Left point turn main module
    :param speed: Motor running speed
    """
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    leftmotor(backward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    IO = 1
    while IO == 1:
        LeftPwm.ChangeDutyCycle(speed)
        RightPwm.ChangeDutyCycle(speed)
        time.sleep(0.01)
        c = GPIO.input(centerled)
        IO = c


def go_forward_any(speed1, speed2):
    """
    Forward module
    :param speed: Motor running speed
    """
    leftmotor(forward0)
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)

    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed1)
    RightPwm.ChangeDutyCycle(speed2)


# def go_backward_any(speed):
#     """
#     Backward module
#     :param speed: Motor running speed
#     """
#     leftmotor(forward0)
#     leftmotor(forward1)
#     GPIO.output(MotorLeft_PWM, GPIO.HIGH)
#     rightmotor(forward0)
#     rightmotor(forward1)
#     GPIO.output(MotorRight_PWM, GPIO.HIGH)
#     LeftPwm.ChangeDutyCycle(speed)
#     RightPwm.ChangeDutyCycle(speed)


def go_forward(speed, running_time):
    """
    Forward module (time limit)
    :param speed: Motor running speed
    :param running_time: Motor running time
    """
    leftmotor(forward0)
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


# def go_backward(speed, running_time):
#     """
#     Backward module (time limit)
#     :param speed: Motor running speed
#     :param running_time: Motor running time
#     """
#     rightmotor(backward0)
#     rightmotor(backward1)
#     GPIO.output(MotorRight_PWM, GPIO.HIGH)
#     leftmotor(backward0)
#     leftmotor(backward1)
#     GPIO.output(MotorLeft_PWM, GPIO.HIGH)
#     LeftPwm.ChangeDutyCycle(speed)
#     RightPwm.ChangeDutyCycle(speed)
#     time.sleep(running_time)


def stop():
    """
    Stop module
    """
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(0)


def pwm_setup():
    """
    Pwm setup module
    """
    LeftPwm.start(0)
    RightPwm.start(0)


# Running pwm setup
pwm_setup()


def pwm_low():
    """
    Pwm low module
    """
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(0)
    GPIO.cleanup()


def linetracing():
    """
    Line tracing module
    0 = black, 1 = white
    """
    a = int(GPIO.input(leftmostled))
    b = int(GPIO.input(leftlessled))
    c = int(GPIO.input(centerled))
    d = int(GPIO.input(rightlessled))
    e = int(GPIO.input(rightmostled))
    stop()
    time.sleep(0.1)
    if (a == 1) & (b == 0) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(33, 38)
    elif (a == 1) & (b == 0) & (c == 1) & (d == 1) & (e == 1):
        go_forward_any(33, 38)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(38, 33)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 0) & (e == 1):
        go_forward_any(38, 33)
    elif (a == 1) & (b == 0) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(38, 38)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(38, 38)
    elif (a == 0) & (b == 1) & (c == 1) & (d == 1) & (e == 1):
        leftPointTurn_any(36)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 1) & (e == 0):
        rightPointTurn_any(36)
    elif (a == 0) & (b == 0) & (c == 1) & (d == 1) & (e == 1):
        leftPointTurn_any(36)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 0) & (e == 0):
        rightPointTurn_any(36)
    elif (a == 0) & (b == 0) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(32, 32)
    elif (a == 0) & (b == 0) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(32, 32)
    elif (a == 0) & (b == 1) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(32, 32)


def determining_cross():
    """
    Determine cross road module
    """
    a = int(GPIO.input(leftmostled))  # d
    b = int(GPIO.input(leftlessled))  # b
    c = int(GPIO.input(centerled))  # a
    d = int(GPIO.input(rightlessled))  # c
    e = int(GPIO.input(rightmostled))  # e
    linetracing()
    time.sleep(0.5)
    if e == 0:
        go_forward(40, 0.45)
        rightPointTurn(37, 0.5)
        rightPointTurn_any(37)
    elif a == 0:
        while a == 1:
            linetracing()
        if c == 0:
            linetracing()
        elif c == 1:
            go_forward(40, 0.45)
            leftPointTurn(37, 0.5)
            leftPointTurn_any(37)
    elif (c == 1) & (b == 1) & (d == 1):
        rightPointTurn_any(42)
    else:
        linetracing()


# 5-way tracking sensor's pin number
dis = 15
leftmostled = 16
leftlessled = 18
centerled = 22
rightlessled = 40
rightmostled = 32

# GPIO input setting
GPIO.setup(leftmostled, GPIO.IN)
GPIO.setup(leftlessled, GPIO.IN)
GPIO.setup(centerled, GPIO.IN)
GPIO.setup(rightlessled, GPIO.IN)
GPIO.setup(rightmostled, GPIO.IN)

# Main
try:
    while True:
        # Debug for 5-way tracking sensor code
        print("\nleftmostled : " + str(GPIO.input(leftmostled)))
        print("leftlessled : " + str(GPIO.input(leftlessled)))
        print("centerled   : " + str(GPIO.input(centerled)))
        print("rightlessled: " + str(GPIO.input(rightlessled)))
        print("rightmostled: " + str(GPIO.input(rightmostled)))
        # Importing the determine way modules code
        determining_cross()


# Keyboard Interrupt
except KeyboardInterrupt:
    # the speed of left motor will be set as LOW
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    # left motor will be stopped with function of ChangeDutyCycle(0)
    LeftPwm.ChangeDutyCycle(0)
    # the speed of right motor will be set as LOW
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    # right motor will be stopped with function of ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(0)
    # GPIO pin setup has been cleared
    GPIO.cleanup()
