import time
import board
import busio
from adafruit_drv8411a import DRV8411A
from bno055 import BNO055

# GPIO / I2C ピン設定
i2c = busio.I2C(board.SCL, board.SDA)

# DRV8411Aの制御ピン（実際の配線に合わせて設定）
# 例: IN1, IN2, IN3, IN4
motor_a_in1 = board.D17
motor_a_in2 = board.D27
motor_b_in1 = board.D22
motor_b_in2 = board.D23

# モータードライバの初期化
driver = DRV8411A(motor_a_in1, motor_a_in2, motor_b_in1, motor_b_in2)

# BNO055の初期化
bno = BNO055(i2c)

def drive(left_speed, right_speed):
    """
    左右のモーター速度を設定する関数
    速度範囲: -1.0 〜 1.0
    """
    driver.motor_a = left_speed
    driver.motor_b = right_speed

def stop():
    """モーターを停止する関数"""
    driver.motor_a = 0
    driver.motor_b = 0

def move(command):
    """コマンドに応じた動作を行う"""
    if command == 'w':
        drive(0.5, 0.5)   # 前進
    elif command == 's':
        drive(-0.5, -0.5) # 後退
    elif command == 'a':
        drive(-0.5, 0.5)  # 左旋回
    elif command == 'd':
        drive(0.5, -0.5)  # 右旋回
    elif command == 'q':
        drive(-0.2, -0.5) # 左後退
    elif command == 'e':
        drive(-0.5, -0.2) # 右後退
    else:
        stop()

def check_stuck():
    """BNO055のデータを使用してスタック検知を行う"""
    try:
        accel = bno.acceleration
        # スタック判定用の簡易ロジック
        if accel is not None:
            # センサ値のチェック処理
            pass
    except Exception as e:
        print(f"BNO055 Read Error: {e}")
