import sys
import time
import RPi.GPIO as GPIO
from gpiozero import Motor
from gpiozero.pins.pigpio import PiGPIOFactory

# ---------------------------------------------------------
# ピン設定 & 定数
# ---------------------------------------------------------
PIN_RIGHT_FORWARD = 18
PIN_RIGHT_BACKWARD = 23
PIN_LEFT_FORWARD = 13
PIN_LEFT_BACKWARD = 24

PIN_LED = 5
PIN_VM = 4

# 安全係数 (8.4V満充電時に6V相当へ抑える)
MAX_POWER_LIMIT = 6.0 / 8.4

motor_right = None
motor_left = None

# ---------------------------------------------------------
# 初期化処理
# ---------------------------------------------------------
def init_hardware():
    global motor_right, motor_left

    # RPi.GPIO 初期化
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_LED, GPIO.OUT)
    GPIO.setup(PIN_VM, GPIO.OUT)
    GPIO.output(PIN_LED, 0)
    GPIO.output(PIN_VM, 1)  # モーター駆動用VMを有効化

    # gpiozero モーター初期化
    try:
        factory = PiGPIOFactory()
        motor_left = Motor(
            forward=PIN_LEFT_FORWARD,
            backward=PIN_LEFT_BACKWARD,
            pin_factory=factory,
        )
        motor_right = Motor(
            forward=PIN_RIGHT_FORWARD,
            backward=PIN_RIGHT_BACKWARD,
            pin_factory=factory,
        )
        print("pigpio 経由でモーターを初期化しました。")
    except Exception as e:
        print(
            f"pigpio接続エラーのため、デフォルト設定で初期化します: {e}"
        )
        motor_left = Motor(
            forward=PIN_LEFT_FORWARD, backward=PIN_LEFT_BACKWARD
        )
        motor_right = Motor(
            forward=PIN_RIGHT_FORWARD, backward=PIN_RIGHT_BACKWARD
        )


# ---------------------------------------------------------
# モーター制御関数
# ---------------------------------------------------------
def set_values(direction, power):
    p = min(1.0, power * MAX_POWER_LIMIT)

    if direction == "w":
        mr, ml = p, p
    elif direction == "s":
        mr, ml = -p, -p
    elif direction == "a":
        mr, ml = -p, p  # 左旋回
    elif direction == "d":
        mr, ml = p, -p  # 右旋回
    elif direction == "q":
        mr, ml = 0, p  # その場左
    elif direction == "e":
        mr, ml = p, 0  # その場右
    else:
        return False

    if motor_right and motor_left:
        motor_right.value = mr
        motor_left.value = ml
    return True


def move(direction, power=0.5, duration=2.0, is_inverted=False):
    if is_inverted:
        mapping = {"w": "s", "s": "w", "a": "a", "d": "d", "q": "e", "e": "q"}
        direction = mapping.get(direction, direction)

    # 緩やかな加速
    steps = 10
    for i in range(steps + 1):
        curr_p = power * (i / steps)
        if not set_values(direction, curr_p):
            return
        time.sleep(0.02)

    # 駆動時間を維持
    time.sleep(max(0.0, duration - 0.4))

    # 緩やかな減速
    for i in range(steps, -1, -1):
        curr_p = power * (i / steps)
        set_values(direction, curr_p)
        time.sleep(0.02)

    # 停止
    if motor_right and motor_left:
        motor_right.value = 0.0
        motor_left.value = 0.0


def cleanup():
    print("\n安全停止処理を行っています...")
    if motor_right:
        motor_right.value = 0.0
        motor_right.close()
    if motor_left:
        motor_left.value = 0.0
        motor_left.close()
    try:
        GPIO.output(PIN_VM, 0)
        GPIO.cleanup()
    except:
        pass
    print("完了したよ。")


# ---------------------------------------------------------
# メインループ
# ---------------------------------------------------------
def main():
    print("=======================================")
    print("  単一ファイル・モーター動作確認スクリプト")
    print("=======================================")
    init_hardware()

    print("\n操作方法:")
    print("  w: 前進, s: 後退, a: 左旋回, d: 右旋回")
    print("  q: 左後退, e: 右後退")
    print("  末尾に 'r' を追加すると逆さま走行 (例: wr)")
    print("  exit または Ctrl+C で終了")

    try:
        while True:
            cmd = input("\nコマンド入力 > ").strip().lower()
            if not cmd or cmd == "exit":
                break

            is_inv = "r" in cmd
            d = cmd.replace("r", "")

            if d in ["w", "s", "a", "d", "q", "e"]:
                print(f"動作実行中: 方向='{d}', 反転={is_inv} (2秒間)")
                move(d, power=0.5, duration=2.0, is_inverted=is_inv)
            else:
                print("無効なコマンドだよ。w/s/a/d/q/e で入力してね。")

    except KeyboardInterrupt:
        print("\nキーボード中断を受け付けたよ。")
    finally:
        cleanup()


if __name__ == "__main__":
    main()
