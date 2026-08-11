import sys
import time

# bno055のインポートによる無限ループ処理を回避するためのダミークラス
class DummyBNO055:
    def __init__(self, *args, **kwargs):
        pass
    def begin(self):
        return True
    def getVector(self, *args, **kwargs):
        return (0.0, 0.0, 0.0)
    def getQuat(self):
        return (1.0, 0.0, 0.0, 0.0)
    def getCalibration(self):
        return (0, 0, 0, 0)
    def getTemp(self):
        return 25

# モジュールキャッシュにダミーを登録して motordrive 内でのインポートを差し替える
sys.modules['bno055'] = type('bno055_module', (), {'BNO055': DummyBNO055})

# ダミー登録後に motordrive をインポート
import motordrive

print("--- モーターテスト開始 ---")

# ---------------------------------------------------------
# 定数・ピン設定
# ---------------------------------------------------------
delta_power = 0.1 # スムーズな加速・減速のための刻み幅
MAX_POWER_LIMIT = 6.0/8.4 #8.4V満充電時に6V相当の電圧にするための安全係数

# DCモータのピン設定 (gpiozero用: BCM番号)
# ※ 実機の配線に合わせて数値を変更してください
PIN_RIGHT_FORWARD = 18 
PIN_RIGHT_BACKWARD = 23 

PIN_LEFT_FORWARD = 13 
PIN_LEFT_BACKWARD = 24 

# その他のGPIOピン (RPi.GPIO用: BCM番号)
PIN_LED = 5
PIN_VM = 4

try:
    print("前進 (w)")
    drive.w(duty=50, drive_time=2)
    time.sleep(1)

    print("後退 (s)")
    drive.s(duty=50, drive_time=2)
    time.sleep(1)

    print("左旋回 (a)")
    drive.a(duty=50, drive_time=2)
    time.sleep(1)

    print("右旋回 (d)")
    drive.d(duty=50, drive_time=2)
    time.sleep(1)

    print("左後退 (q)")
    drive.q(duty=50, drive_time=2)
    time.sleep(1)

    print("右後退 (e)")
    drive.e(duty=50, drive_time=2)
    time.sleep(1)

    print("--- テスト完了 ---")

except KeyboardInterrupt:
    print("中断されました。")
finally:
    drive.cleanup()
