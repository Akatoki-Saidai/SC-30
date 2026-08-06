#---------------------------------------------------------------------
# TB67H450 Motor Driver
# Raspberry Pi + gpiozero + pigpio
#
# Right Motor
#   IN1 : GPIO18
#   IN2 : GPIO23
#
# Left Motor
#   IN1 : GPIO13
#   IN2 : GPIO24
#
# nSLEEP : GPIO4
#---------------------------------------------------------------------

from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory
import RPi.GPIO as GPIO


class MotorDriver:

    def __init__(self):

        # GPIO設定
        self.PIN_RIGHT_IN1 = 18
        self.PIN_RIGHT_IN2 = 23

        self.PIN_LEFT_IN1 = 13
        self.PIN_LEFT_IN2 = 24

        self.PIN_NSLEEP = 4

        # pigpio
        self.factory = PiGPIOFactory()

        # GPIO初期化
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PIN_NSLEEP, GPIO.OUT)

        # Driver Enable
        GPIO.output(self.PIN_NSLEEP, GPIO.HIGH)

        # Motor生成
        self.right_motor = Motor(
            forward=self.PIN_RIGHT_IN1,
            backward=self.PIN_RIGHT_IN2,
            pin_factory=self.factory
        )

        self.left_motor = Motor(
            forward=self.PIN_LEFT_IN1,
            backward=self.PIN_LEFT_IN2,
            pin_factory=self.factory
        )

    #---------------------------------------------------------
    # 速度制限
    #---------------------------------------------------------
    def limit_speed(self, speed):

        if speed > 1.0:
            speed = 1.0

        if speed < 0.0:
            speed = 0.0

        return speed

    #---------------------------------------------------------
    # 前進
    #---------------------------------------------------------
    def forward(self, speed):

        speed = self.limit_speed(speed)

        self.right_motor.forward(speed)
        self.left_motor.forward(speed)

    #---------------------------------------------------------
    # 後進
    #---------------------------------------------------------
    def backward(self, speed):

        speed = self.limit_speed(speed)

        self.right_motor.backward(speed)
        self.left_motor.backward(speed)

    #---------------------------------------------------------
    # 左回転
    #---------------------------------------------------------
    def turn_left(self, speed):

        speed = self.limit_speed(speed)

        self.right_motor.forward(speed)
        self.left_motor.backward(speed)

    #---------------------------------------------------------
    # 右回転
    #---------------------------------------------------------
    def turn_right(self, speed):

        speed = self.limit_speed(speed)

        self.right_motor.backward(speed)
        self.left_motor.forward(speed)

    #---------------------------------------------------------
    # 停止
    #---------------------------------------------------------
    def stop(self):

        self.right_motor.stop()
        self.left_motor.stop()

    #---------------------------------------------------------
    # 終了処理
    #---------------------------------------------------------
    def cleanup(self):

        self.stop()

        GPIO.output(self.PIN_NSLEEP, GPIO.LOW)

        self.right_motor.close()
        self.left_motor.close()

        GPIO.cleanup()