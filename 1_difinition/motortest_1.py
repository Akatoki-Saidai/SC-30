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

# DRV8411A の初期化 (GPIOピンは環境に合わせて設定)
# 例: Left IN1=17, IN2=27, Right IN1=22, IN2=23
drive = motordrive.DRV8411A(pin_left_in1=17, pin_left_in2=27, pin_right_in1=22, pin_right_in2=23)

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
