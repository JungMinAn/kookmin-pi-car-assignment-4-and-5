"""
# Date: 2017/11/06
# file name: picar-assignment4.py
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


def getDistance():
    """
    Detect distance
    :return: distance
    """
    GPIO.output(trig, False)
    time.sleep(0.5)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 2)
    return distance


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
    if (a == 0) & (b == 0) & (c == 0) & (d == 0) & (e == 0):
        go_forward_any(0, 0)
    elif (a == 0) & (b == 0) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(20, 0)
    elif (a == 0) & (b == 0) & (c == 0) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 0) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(45, 35)
    elif (a == 0) & (b == 0) & (c == 1) & (d == 0) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 0) & (c == 1) & (d == 0) & (e == 1):
        time.sleep(1)
    elif (a == 0) & (b == 0) & (c == 1) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 0) & (c == 1) & (d == 1) & (e == 1):
        go_forward_any(13, 50)
    elif (a == 0) & (b == 1) & (c == 0) & (d == 0) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 0) & (d == 0) & (e == 1):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 0) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 0) & (d == 1) & (e == 1):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 1) & (d == 0) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 1) & (d == 0) & (e == 1):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 1) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 0) & (b == 1) & (c == 1) & (d == 1) & (e == 1):
        go_forward_any(12, 50)
    elif (a == 1) & (b == 0) & (c == 0) & (d == 0) & (e == 0):
        go_forward_any(0, 20)
    elif (a == 1) & (b == 0) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(35, 35)
    elif (a == 1) & (b == 0) & (c == 0) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 1) & (b == 0) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(30, 35)
    elif (a == 1) & (b == 0) & (c == 1) & (d == 0) & (e == 0):
        time.sleep(1)
    elif (a == 1) & (b == 0) & (c == 1) & (d == 0) & (e == 1):
        time.sleep(1)
    elif (a == 1) & (b == 0) & (c == 1) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 1) & (b == 0) & (c == 1) & (d == 1) & (e == 1):
        go_forward_any(25, 35)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 0) & (e == 0):
        go_forward_any(50, 35)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 0) & (e == 1):
        go_forward_any(40, 35)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 1) & (e == 0):
        time.sleep(1)
    elif (a == 1) & (b == 1) & (c == 0) & (d == 1) & (e == 1):
        go_forward_any(35, 35)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 0) & (e == 0):
        go_forward_any(50, 13)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 0) & (e == 1):
        go_forward_any(35, 30)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 1) & (e == 0):
        go_forward_any(40, 27)
    elif (a == 1) & (b == 1) & (c == 1) & (d == 1) & (e == 1):
        go_forward_any(40, 0)


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
        print("leftmostled  detects black line(0) or white ground(1): " + str(GPIO.input(leftmostled)))
        print("leftlessled  detects black line(0) or white ground(1): " + str(GPIO.input(leftlessled)))
        print("centerled    detects black line(0) or white ground(1): " + str(GPIO.input(centerled)))
        print("rightlessled detects black line(0) or white ground(1): " + str(GPIO.input(rightlessled)))
        print("rightmostled detects black line(0) or white ground(1): " + str(GPIO.input(rightmostled)))
        distance = getDistance()
        print('distance= ', distance)

        # when the distance is above the dis, moving object forwards
        if (distance > dis):
            linetracing()

        # when the distance is below the dis, moving object stops
        else:
            stop()
            time.sleep(1)
            rightPointTurn(34, 0.38)
            time.sleep(1)
            go_forward(38, 0.3)
            time.sleep(1)
            leftPointTurn(33, 0.35)
            time.sleep(1)
            go_forward(42, 0.8)
            time.sleep(1)
            leftPointTurn(31, 0.35)
            time.sleep(1)
            go_forward(30, 0.28)
            time.sleep(1)
            rightPointTurn(28, 0.35)

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
