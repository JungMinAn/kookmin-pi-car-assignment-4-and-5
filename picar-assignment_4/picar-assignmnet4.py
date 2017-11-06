#########################################################################
# Date: 2017/11/06
# file name: picar-assignment4.py
#########################################################################

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

pwm_setup()

trig = 33
echo = 31

GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)


def getDistance():
    GPIO.output(trig, False)
    time.sleep(0.5)
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end-pulse_start
    distance = pulse_duration*17000
    distance = round(distance, 2)
    return distance


def REVERSE(x):
    if x == True:
        return False
    elif x == False:
        return True


forward0 = True
forward1 = False

backward0 = REVERSE(forward0)
backward1 = REVERSE(forward1)

MotorLeft_A = 12
MotorLeft_B = 11
MotorLeft_PWM = 35

MotorRight_A = 15
MotorRight_B = 13
MotorRight_PWM = 37


def leftmotor(x):
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
    if x == True:
        GPIO.output(MotorRight_A, GPIO.LOW)
        GPIO.output(MotorRight_B, GPIO.HIGH)
    elif x == False:
        GPIO.output(MotorRight_A, GPIO.HIGH)
        GPIO.output(MotorRight_B, GPIO.LOW)


GPIO.setup(MotorLeft_A, GPIO.OUT)
GPIO.setup(MotorLeft_B, GPIO.OUT)
GPIO.setup(MotorLeft_PWM, GPIO.OUT)

GPIO.setup(MotorRight_A, GPIO.OUT)
GPIO.setup(MotorRight_B, GPIO.OUT)
GPIO.setup(MotorRight_PWM, GPIO.OUT)

LeftPwm = GPIO.PWM(MotorLeft_PWM, 100)
RightPwm = GPIO.PWM(MotorRight_PWM, 100)


def rightSwingTurn(speed, running_time):
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(0)
    time.sleep(running_time)


def leftSwingTurn(speed, running_time):
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def rightPointTurn(speed, running_time):
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(backward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def leftPointTurn(speed, running_time):
    rightmotor(forward0)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    leftmotor(backward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def go_forward_any(speed):
    leftmotor(forward0)
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)

    rightmotor(forward0)
    rightmotor(forward1)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)


def go_backward_any(speed):
    leftmotor(forward0)
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(forward0)
    rightmotor(forward1)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)


def go_forward(speed, running_time):
    leftmotor(forward0)
    leftmotor(forward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    rightmotor(forward0)
    rightmotor(forward1)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def go_backward(speed, running_time):
    rightmotor(backward0)
    rightmotor(backward1)
    GPIO.output(MotorRight_PWM, GPIO.HIGH)
    leftmotor(backward0)
    leftmotor(backward1)
    GPIO.output(MotorLeft_PWM, GPIO.HIGH)
    LeftPwm.ChangeDutyCycle(speed)
    RightPwm.ChangeDutyCycle(speed)
    time.sleep(running_time)


def stop():
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(0)


def pwm_setup():
    LeftPwm.start(0)
    RightPwm.start(0)


def pwm_low():
    GPIO.output(MotorLeft_PWM, GPIO.LOW)
    GPIO.output(MotorRight_PWM, GPIO.LOW)
    LeftPwm.ChangeDutyCycle(0)
    RightPwm.ChangeDutyCycle(0)
    GPIO.cleanup()

# =======================================================================
# declare the pins of 16, 18, 22, 40, 32 in the Rapberry Pi
# as the control pins of 5-way trackinmg sensor in order to
# control direction
#
#  leftmostled    leftlessled     centerled     rightlessled     rightmostled
#       16            18              22             40              32
#
# led turns on (1) : trackinmg sensor led detects white playground
# led turns off(0) : trackinmg sensor led detects black line

# leftmostled off : it means that moving object finds black line
#                   at the position of leftmostled
#                   black line locates below the leftmostled of the moving object
#
# leftlessled off : it means that moving object finds black line
#                   at the position of leftlessled
#                   black line locates below the leftlessled of the moving object
#
# centerled off : it means that moving object finds black line
#                   at the position of centerled
#                   black line locates below the centerled of the moving object
#
# rightlessled off : it means that moving object finds black line
#                   at the position of rightlessled
#                   black line locates below the rightlessled  of the moving object
#
# rightmostled off : it means that moving object finds black line
#                   at the position of rightmostled
#                   black line locates below the rightmostled of the moving object
# =======================================================================

leftmostled = 16
leftlessled = 18
centerled = 22
rightlessled = 40
rightmostled = 32

# =======================================================================
# because the connetions between 5-way tracking sensor and Rapberry Pi has been
# established, the GPIO pins of Rapberry Pi
# such as leftmostled, leftlessled, centerled, rightlessled, and rightmostled
# should be clearly declared whether their roles of pins
# are output pin or input pin
# since the 5-way tracking sensor data has been detected and
# used as the input data, leftmostled, leftlessled, centerled, rightlessled, and rightmostled
# should be clearly declared as input
#
# =======================================================================

GPIO.setup(leftmostled, GPIO.IN)
GPIO.setup(leftlessled, GPIO.IN)
GPIO.setup(centerled, GPIO.IN)
GPIO.setup(rightlessled, GPIO.IN)
GPIO.setup(rightmostled, GPIO.IN)

# =======================================================================
# GPIO.input(leftmostled) method gives the data obtained from leftmostled
# leftmostled returns (1) : leftmostled detects white playground
# leftmostled returns (0) : leftmostled detects black line
#
#
# GPIO.input(leftlessled) method gives the data obtained from leftlessled
# leftlessled returns (1) : leftlessled detects white playground
# leftlessled returns (0) : leftlessled detects black line
#
# GPIO.input(centerled) method gives the data obtained from centerled
# centerled returns (1) : centerled detects white playground
# centerled returns (0) : centerled detects black line
#
# GPIO.input(rightlessled) method gives the data obtained from rightlessled
# rightlessled returns (1) : rightlessled detects white playground
# rightlessled returns (0) : rightlessled detects black line
#
# GPIO.input(rightmostled) method gives the data obtained from rightmostled
# rightmostled returns (1) : rightmostled detects white playground
# rightmostled returns (0) : rightmostled detects black line
#
# =======================================================================

# we need this part!!!
try:
    while True:
        print("leftmostled  detects black line(0) or white ground(1): " + str(GPIO.input(leftmostled)))
        print("leftlessled  detects black line(0) or white ground(1): " + str(GPIO.input(leftlessled)))
        print("centerled    detects black line(0) or white ground(1): " + str(GPIO.input(centerled)))
        print("rightlessled detects black line(0) or white ground(1): " + str(GPIO.input(rightlessled)))
        print("rightmostled detects black line(0) or white ground(1): " + str(GPIO.input(rightmostled)))
        time.sleep(1)

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




