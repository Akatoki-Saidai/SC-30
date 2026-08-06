#---------------------------------------------------------------------
# TB67H450 Motor Driver
# Raspberry Pi + gpiozero + pigpio
#
# IN1 / IN2 H-Bridge control
#
# Right Motor:
#   IN1 GPIO18
#   IN2 GPIO23
#
# Left Motor:
#   IN1 GPIO13
#   IN2 GPIO24
#
# Function:
#   forward(speed)
#   backward(speed)
#   turn_left(speed)
#   turn_right(speed)
#   stop()
#---------------------------------------------------------------------

from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import time
import RPi.GPIO as GPIO



# ---------------------------------------------------------
# GPIO設定
# ---------------------------------------------------------

PIN_RIGHT_IN1 = 18
PIN_RIGHT_IN2 = 23

PIN_LEFT_IN1 = 13
PIN_LEFT_IN2 = 24


# TB67H450 nSLEEP
PIN_NSLEEP = 4



# ---------------------------------------------------------
# 初期化
# ---------------------------------------------------------

_factory = PiGPIOFactory()


GPIO.setmode(GPIO.BCM)

GPIO.setup(
    PIN_NSLEEP,
    GPIO.OUT
)

# Driver Enable
GPIO.output(
    PIN_NSLEEP,
    GPIO.HIGH
)



# ---------------------------------------------------------
# Motor生成
# ---------------------------------------------------------

right_motor = Motor(
    forward=PIN_RIGHT_IN1,
    backward=PIN_RIGHT_IN2,
    pin_factory=_factory
)


left_motor = Motor(
    forward=PIN_LEFT_IN1,
    backward=PIN_LEFT_IN2,
    pin_factory=_factory
)



# ---------------------------------------------------------
# 速度制限
# ---------------------------------------------------------

def limit_speed(speed):

    if speed > 1.0:
        speed = 1.0

    if speed < 0:
        speed = 0

    return speed



# ---------------------------------------------------------
# 前進
# ---------------------------------------------------------

def forward(speed):

    speed = limit_speed(speed)

    right_motor.forward(speed)
    left_motor.forward(speed)



# ---------------------------------------------------------
# 後進
# ---------------------------------------------------------

def backward(speed):

    speed = limit_speed(speed)

    right_motor.backward(speed)
    left_motor.backward(speed)



# ---------------------------------------------------------
# 左回転
# ---------------------------------------------------------

def turn_left(speed):

    speed = limit_speed(speed)

    # 左車輪後退
    # 右車輪前進

    right_motor.forward(speed)
    left_motor.backward(speed)



# ---------------------------------------------------------
# 右回転
# ---------------------------------------------------------

def turn_right(speed):

    speed = limit_speed(speed)

    # 左車輪前進
    # 右車輪後退

    right_motor.backward(speed)
    left_motor.forward(speed)



# ---------------------------------------------------------
# 停止
# ---------------------------------------------------------

def stop():

    right_motor.stop()
    left_motor.stop()



# ---------------------------------------------------------
# 終了処理
# ---------------------------------------------------------

def cleanup():

    stop()

    GPIO.output(
        PIN_NSLEEP,
        GPIO.LOW
    )

    right_motor.close()
    left_motor.close()

    GPIO.cleanup()



# ---------------------------------------------------------
# 動作テスト
# ---------------------------------------------------------

if __name__ == "__main__":


    try:

        print("TB67H450 Motor Test")

        while True:


            print()
            print("w : forward")
            print("s : backward")
            print("a : turn left")
            print("d : turn right")
            print("x : stop")
            print("q : quit")


            cmd = input("> ")


            if cmd == "w":

                speed = float(
                    input("speed 0-1 : ")
                )

                forward(speed)

                time.sleep(2)

                stop()



            elif cmd == "s":

                speed = float(
                    input("speed 0-1 : ")
                )

                backward(speed)

                time.sleep(2)

                stop()



            elif cmd == "a":

                speed = float(
                    input("speed 0-1 : ")
                )

                turn_left(speed)

                time.sleep(2)

                stop()



            elif cmd == "d":

                speed = float(
                    input("speed 0-1 : ")
                )

                turn_right(speed)

                time.sleep(2)

                stop()



            elif cmd == "x":

                stop()



            elif cmd == "q":

                break



            else:

                print("Invalid command")



    except KeyboardInterrupt:

        pass


    finally:

        cleanup()