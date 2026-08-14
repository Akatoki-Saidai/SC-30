# ============================================================
# EM.py (SC-30 統合版)
# ------------------------------------------------------------
# phase1&2.py（待機・落下フェーズ）
# phase3.py  （遠距離フェーズ）
# phase4.py  （近距離フェーズ）
# を、各フェーズのロジックには手を加えずにそのまま縦に連結したもの。
# SC-28/4_FM/FM.py の構成（1つのmain()の中でphase変数を1→2→3→4→5と
# 進めていく形）にならって、3つのファイルの中身を1本にまとめてある。
#
# ★このファイルで元コードに対して行った“最小限の接着”のみ、以下にメモしておく
#   （ロジック・数値・処理内容は一切変更していない）
#   1. phase1&2.py / phase3.py / phase4.py はそれぞれ独立した main() を
#      持っていたため、1つの main() ＋ phase 変数の while ループに統合した。
#      各フェーズの中身（if/elif phase == N: 以下）はそれぞれの元ファイルの
#      該当ブロックをそのまま移植している。
#   2. phase3.py の run_long_distance_phase() は元々 phase3.py の main() から
#      `phase = run_long_distance_phase(...)` として呼ばれていたので、
#      それと全く同じ形で phase==3 ブロックから呼び出している（関数の中身は無変更）。
# ============================================================

import time
import cv2
import sys
import math
import threading
import numpy as np
import RPi.GPIO as GPIO

# --- ピン設定 ---
LED_PIN = 5
NICHROME_PIN = 16

# ==========================================
# 個別モジュール読み込み（失敗したら None を代入）
# ==========================================
# 各センサ／モジュールは実機環境でのみ利用可能な依存があるため、
# インポート時に失敗しても他の機能をテストできるように個別にガードします。

# --- phase1&2.py が期待していたカメラ import ---
try:
    from camera_sc30 import Camera
except Exception as e:
    Camera = None
    print(f"Camera import error: {e}")

try:
    from bno055 import BNO055
except Exception as e:
    BNO055 = None
    print(f"BNO055 import error: {e}")

try:
    from bme280 import BME280Sensor
except Exception as e:
    BME280Sensor = None
    print(f"BME280 import error: {e}")

try:
    from gps import idokeido, calculate_distance_and_angle, ERROR_DISTANCE
except Exception as e:
    idokeido = None
    calculate_distance_and_angle = None
    ERROR_DISTANCE = None
    print(f"GPS import error: {e}")

try:
    import motordrive as md
except Exception as e:
    md = None
    print(f"motordrive import error: {e}")

# make_csv を読み込んでログ保存できるようにする
try:
    import make_csv
except Exception as e:
    make_csv = None
    print(f"make_csv import error: {e}")

# --- phase4.py が使っていたカメラ import（SC-30実機のカメラモジュール） ---
try:
    from camera_sc30 import Camera
except Exception as e:
    print(f"camera_sc30 import error: {e}")

# ログ用のヘルパー
def log_msg(msg_type, msg_data):
    try:
        if make_csv is not None:
            make_csv.print(msg_type, msg_data)
    except Exception:
        # ロギングは補助的な処理。失敗しても主処理を止めない。
        pass


# ============================================================
# ------------------------------------------------------------
# phase3.py 由来の設定値
# ------------------------------------------------------------
# ============================================================

# ------------------------------------------------------------
# ゴール座標
# 実際のゴール座標を設定する
# ------------------------------------------------------------
GOAL_LAT = 40.14262816666667 # 本番
GOAL_LON = 139.987715

# ------------------------------------------------------------
# ゴール判定
# ------------------------------------------------------------
GOAL_THRESHOLD_M = 10.0

# ------------------------------------------------------------
# モーター
# ------------------------------------------------------------
POWER = 0.7

# サブキャリア離脱
INITIAL_FORWARD_TIME = 5.0

# 通常前進
FORWARD_TIME = 15

# ------------------------------------------------------------
# 旋回
# ------------------------------------------------------------
# 1秒間に約90°旋回する想定
OMEGA_DEG_PER_SEC = 180.0

# ゴール方向との差が15°以内なら旋回不要
TURN_TOLERANCE_DEG = 15.0

# 最低旋回時間
MIN_TURN_TIME = 0.3

# 1回の最大旋回時間
MAX_TURN_TIME = 4.0

# BNO055による旋回補正回数
MAX_TURN_ATTEMPTS = 3

# ------------------------------------------------------------
# スタック検知
# ------------------------------------------------------------
# ★実験によって決める必要がある
STACK_ACCEL_THRESHOLD = 0.5  # m/s^2

# ax, ay, az がすべて閾値以下の状態が1秒続いたらスタック
STACK_HOLD_TIME = 1.0

# 加速度を読む間隔
STACK_SAMPLE_TIME = 0.05

# ------------------------------------------------------------
# スタック復帰
# ------------------------------------------------------------
# 3秒後退
BACK_TIME = 3.0

# 60°右旋回
RECOVERY_TURN_DEG = 60.0

# その後2秒前進
RECOVERY_FORWARD_TIME = 2.0


# ------------------------------------------------------------
# GPS
# ------------------------------------------------------------
GPS_FAIL_LIMIT = 6



# ============================================================
# 左右旋回コマンド
# ============================================================
LEFT_TURN_CMD = "q"
RIGHT_TURN_CMD = "e"


# ==========================================
# セットアップ（phase1&2.py 由来。このEM.pyではこちらを使用する）
# ==========================================
def setup_sensors():

    print("bnoセットアップ開始")
    log_msg('msg', 'bnoセットアップ開始')
    bno = None
    try:
        if BNO055 is not None:
            bno = BNO055()
            if not bno.begin():
                print("BNO055: Init Failed")
                log_msg('error', 'BNO055: Init Failed')
                bno = None
        else:
            print("BNO055 モジュールが利用できません（インポート失敗）")
            log_msg('error', 'BNO055 import failed')
    except Exception as e:
        print(f"BNO055 Setup Error: {e}")
        log_msg('error', f"BNO055 Setup Error: {e}")

    print("cameraセットアップ開始")
    log_msg('msg', 'cameraセットアップ開始')
    cam = None
    try:
        if Camera is not None:
            cam = Camera(model_path="./my_custom_model.pt", debug=True)
        else:
            print("Camera モジュールが利用できません（インポート失敗）")
            log_msg('error', 'Camera import failed')
    except Exception as e:
        print(f"Camera Setup Error: {e}")
        log_msg('error', f"Camera Setup Error: {e}")

    print("bmeセットアップ開始")
    log_msg('msg', 'bmeセットアップ開始')
    bme = None
    qnh = 1013.25
    try:
        if BME280Sensor is not None:
            bme = BME280Sensor(debug=False)
            if bme.calib_ok:
                qnh = bme.baseline()
            else:
                print("BME280: Calibration Failed")
                log_msg('error', 'BME280: Calibration Failed')
                bme = None
        else:
            print("BME280 モジュールが利用できません（インポート失敗）")
            log_msg('error', 'BME280 import failed')
    except Exception as e:
        print(f"BME280 Setup Error: {e}")
        log_msg('error', f"BME280 Setup Error: {e}")

    print("モータセットアップ開始")
    log_msg('msg', 'モータセットアップ開始')
    motor_ok = False
    try:
        if md is not None:
            md.setup_motors()
            motor_ok = True
        else:
            print("motordrive モジュールが利用できません（インポート失敗）")
            log_msg('error', 'motordrive import failed')
    except Exception as e:
        print(f"Motor Setup Error: {e}")
        log_msg('error', f"Motor Setup Error: {e}")

    print("GPIOセットアップ開始")
    log_msg('msg', 'GPIOセットアップ開始')
    gpio_ok = False
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_PIN, GPIO.OUT)
        GPIO.setup(NICHROME_PIN, GPIO.OUT)
        GPIO.output(LED_PIN, 0)
        GPIO.output(NICHROME_PIN, 0)
        gpio_ok = True
    except Exception as e:
        print(f"GPIO Setup Error: {e}")
        log_msg('error', f"GPIO Setup Error: {e}")

    return bno, cam, bme, qnh, motor_ok, gpio_ok


# ============================================================
# ------------------------------------------------------------
# phase3.py 由来の関数群（中身は無変更）
# ------------------------------------------------------------
# ============================================================

# ============================================================
# 角度正規化
# ============================================================
def normalize_angle(angle_deg):
    """角度を -180 ～ +180° に変換"""
    return (angle_deg + 180.0) % 360.0 - 180.0


# ============================================================
# 初回センサデータ取得
# ============================================================
def get_initial_sensor_data(bno):
    """
    遠距離フェーズ開始直後。
    フローチャート通り
    ・GPS / 加速度 / 重力加速度 / 角速度 / 地磁気 を取得する。
    """
    lat, lon = idokeido()

    if bno is None:
        return {
            "lat": lat,
            "lon": lon,
            "accel": None,
            "gravity": None,
            "gyro": None,
            "mag": None,
        }

    return {
        "lat": lat,
        "lon": lon,
        "accel": bno.linear_acceleration(),
        "gravity": bno.gravity(),
        "gyro": bno.gyroscope(),
        "mag": bno.magnetometer(),
    }


# ============================================================
# 遠距離走行中のセンサデータ取得
# ============================================================
def get_running_sensor_data(bno):
    """
    初期姿勢確認後の遠距離ループ。
    フローチャート通り
    ・GPS / 加速度 / 角速度 / 地磁気 のみ取得。
    ★重力加速度はここでは取得しない。
    ★機体反転判定もここでは行わない。
    """
    lat, lon = idokeido()

    if bno is None:
        return {
            "lat": lat,
            "lon": lon,
            "accel": None,
            "gyro": None,
            "mag": None,
        }

    return {
        "lat": lat,
        "lon": lon,
        "accel": bno.linear_acceleration(),
        "gyro": bno.gyroscope(),
        "mag": bno.magnetometer(),
    }


# ============================================================
# 機体姿勢復帰
# ============================================================
def correct_orientation():
    """
    機体が反転していた場合に正しい姿勢へ戻すための動作。
    """
    try:
        print("Starting Inverted Release Sequence...")
        # LED点滅
        for _ in range(3):
            GPIO.output(PIN_LED, 1)
            time.sleep(0.5)
            GPIO.output(PIN_LED, 0)
            time.sleep(0.5)

        # もがき動作
        # 1. 前進 (10秒)
        md.move('w', 1.0, 10.0, is_inverted=True, enable_stack_check=False)
            
        md.stop()
        print("Inverted Release Sequence Finished.")
            
    except Exception as e:
        print(f"Error in check_stuck (inverted): {e}")

    raise NotImplementedError(
        "機体姿勢復帰時のモーター動作が未設定です。"
        "機体を裏返し状態から正常姿勢へ戻す"
        "具体的なモーター動作をcorrect_orientation()に設定してください。"
    )


# ============================================================
# ゴール方向への旋回
# ============================================================
def turn_by_angle(bno, angle_deg, motor_ok=True):
    """
    GPSから求めた相対角度だけ旋回する。
    gps.pyでは angle > 0 → 左, angle < 0 → 右
    """
    if not motor_ok:
        print("モーターが使用できないため旋回できません")
        return

    # 15°以内なら旋回しない
    if abs(angle_deg) <= TURN_TOLERANCE_DEG:
        print("方向差15°以内のため旋回不要")
        return

    # BNO055が使えない場合
    cmd = LEFT_TURN_CMD if angle_deg > 0 else RIGHT_TURN_CMD
    turn_time = abs(angle_deg) / OMEGA_DEG_PER_SEC
    turn_time = max(turn_time, MIN_TURN_TIME)
    turn_time = min(turn_time, MAX_TURN_TIME)

    md.move(
        cmd,
        power=POWER,
        duration=turn_time,
        is_inverted=False,
        enable_stack_check=False
    )
    return

    # # BNO055フィードバック旋回
    # euler = bno.euler()
    # if euler is None:
    #     print("BNO055 Euler角取得失敗")
    #     return

    # start_yaw = euler[0]
    # target_yaw = (start_yaw + angle_deg) % 360.0

    # print(f"旋回開始：現在Yaw={start_yaw:.1f}° / 目標Yaw={target_yaw:.1f}°")

    # # 最大3回補正
    # for attempt in range(MAX_TURN_ATTEMPTS):
    #     euler = bno.euler()
    #     if euler is None:
    #         print("Yaw取得失敗")
    #         break

    #     current_yaw = euler[0]
    #     diff = normalize_angle(target_yaw - current_yaw)

    #     print(f"現在Yaw={current_yaw:.1f}° / 残り={diff:.1f}°")

    #     if abs(diff) <= TURN_TOLERANCE_DEG:
    #         print("旋回完了")
    #         return

    #     cmd = LEFT_TURN_CMD if diff > 0 else RIGHT_TURN_CMD
    #     turn_time = abs(diff) / OMEGA_DEG_PER_SEC
    #     turn_time = max(turn_time, MIN_TURN_TIME)
    #     turn_time = min(turn_time, MAX_TURN_TIME)

    #     print(f"旋回補正 {attempt + 1}/{MAX_TURN_ATTEMPTS} {turn_time:.2f}s")

    #     md.move(
    #         cmd,
    #         power=POWER,
    #         duration=turn_time,
    #         is_inverted=False,
    #         enable_stack_check=False
    #     )
    #    time.sleep(0.3)
    #print(f"ターゲットYaw{target_yaw} / 現在Yaw{current_yaw}")


# ============================================================
# 前進 + スタック検知
# ============================================================
def forward_with_stack_check(bno, duration, accel_threshold, motor_ok=True):
    """
    モーターを稼働させて ax, ay, az が閾値以下の状態が1秒継続するか判定。
    BNO055.linear_acceleration() を使用。
    """
    if not motor_ok:
        print("モーターが使用できません")
        return False

    if bno is None:
        raise RuntimeError("スタック判定にはBNO055が必要です")

    if accel_threshold is None:
        raise ValueError("STACK_ACCEL_THRESHOLDが未設定です")

    if accel_threshold <= 0:
        raise ValueError("STACK_ACCEL_THRESHOLDは正の値にしてください")

    motor_error = []

    def motor_task():
        try:
            md.move(
                "w",
                power=POWER,
                duration=duration,
                is_inverted=False,
                enable_stack_check=False
            )
        except Exception as e:
            motor_error.append(e)

    motor_thread = threading.Thread(target=motor_task, daemon=True)
    motor_thread.start()

    low_accel_start = None
    stacked = False

    while motor_thread.is_alive():
        accel = bno.linear_acceleration()

        if accel is None:
            low_accel_start = None
            time.sleep(STACK_SAMPLE_TIME)
            continue

        ax, ay, az = accel
        #print(f"Linear Accel x={ax:.2f}, y={ay:.2f}, z={az:.2f}")

        low_accel = (
            abs(ax) <= accel_threshold and
            abs(ay) <= accel_threshold and
            abs(az) <= accel_threshold
        )

        if low_accel:
            if low_accel_start is None:
                low_accel_start = time.monotonic()
            elif time.monotonic() - low_accel_start >= STACK_HOLD_TIME:
                stacked = True
        else:
            low_accel_start = None

        time.sleep(STACK_SAMPLE_TIME)

    motor_thread.join()

    if motor_error:
        raise motor_error[0]

    return stacked


# ============================================================
# スタック復帰
# ============================================================
def recover_from_stuck(bno, motor_ok=True):
    """
    スタック復帰処理：3秒後退 ➔ 60°右旋回 ➔ 2秒前進
    """
    if not motor_ok:
        return None, None

    print("\n====================")
    print(" スタック復帰開始")
    print("====================")

    # ① 3秒後退
    print("3秒後退")
    md.move(
        "s",
        power=POWER,
        duration=BACK_TIME,
        is_inverted=False,
        enable_stack_check=False
    )

    # ② 60°右旋回 (-60°)
    print("60°右旋回")
    turn_by_angle(
        bno=bno,
        angle_deg=-RECOVERY_TURN_DEG,
        motor_ok=motor_ok
    )

    # ③ 2秒前進する直前のGPS
    start_lat, start_lon = idokeido()

    # ④ 2秒前進
    print("2秒前進")
    md.move(
        "w",
        power=POWER,
        duration=RECOVERY_FORWARD_TIME,
        is_inverted=False,
        enable_stack_check=False
    )

    time.sleep(0.5)
    print("スタック復帰終了")

    return start_lat, start_lon


# ============================================================
# 遠距離フェーズ本体
# ============================================================
def run_long_distance_phase(bno, goal_lat, goal_lon, stack_accel_threshold, motor_ok=True):
    print("\n==========================")
    print(" 遠距離フェーズ開始")
    print("==========================")

    # 初期姿勢確認ループ
    while True:
        print("\n--- 初期センサ取得 ---")
        data = get_initial_sensor_data(bno)

        curr_lat = data["lat"]
        curr_lon = data["lon"]
        accel = data["accel"]
        gravity = data["gravity"]
        gyro = data["gyro"]
        mag = data["mag"]

        if curr_lat is not None and curr_lon is not None:
            print(f"GPS: {curr_lat}, {curr_lon}")
        else:
            print("GPS取得失敗")

        if accel is not None:
            print(f"加速度: {accel}")

        if gravity is not None:
            print(f"重力加速度: {gravity}")
        else:
            print("重力加速度取得失敗")

        if gyro is not None:
            print(f"角速度: {gyro}")

        if mag is not None:
            print(f"地磁気: {mag}")

        if curr_lat is None or curr_lon is None:
            time.sleep(1.0)
            continue

        if gravity is None:
            time.sleep(0.5)
            continue

        if gravity[2] < 0:
            print("重力z < 0\n機体は正常な向きです")
            break

        print("重力z >= 0\n機体の反転を検知\n機体姿勢復帰を開始します")
        md.move('q', power=0.7, duration=0.5, is_inverted=True, enable_stack_check=True)
        break

    prev_lat = curr_lat
    prev_lon = curr_lon

    # サブキャリア離脱のため5秒前進
    print("\nサブキャリア離脱のため5秒前進")
    if motor_ok:
        md.move(
            "w",
            power=POWER,
            duration=INITIAL_FORWARD_TIME,
            is_inverted=False,
            enable_stack_check=False
        )

    time.sleep(1.0)
    gps_fail_count = 0

    # 遠距離メインループ
    while True:
        print("\n--- 遠距離走行ループ ---")
        data = get_running_sensor_data(bno)

        curr_lat = data["lat"]
        curr_lon = data["lon"]
        accel = data["accel"]
        gyro = data["gyro"]
        mag = data["mag"]

        if curr_lat is None or curr_lon is None:
            gps_fail_count += 1
            print(f"GPS取得失敗 {gps_fail_count}/{GPS_FAIL_LIMIT}")

            if gps_fail_count >= GPS_FAIL_LIMIT:
                print("GPS取得失敗が連続しました\n近距離フェーズへ移行")
                return 4

            time.sleep(1.0)
            continue

        gps_fail_count = 0
        print(f"GPS: {curr_lat}, {curr_lon}")

        distance_m, angle_rad = calculate_distance_and_angle(
            curr_lat, curr_lon,
            prev_lat, prev_lon,
            goal_lat, goal_lon
        )

        if distance_m == ERROR_DISTANCE:
            print("距離・角度計算失敗")
            time.sleep(0.5)
            continue

        angle_deg = math.degrees(angle_rad)
        print(f"ゴールまでの距離: {distance_m:.2f} m")
        print(f"ゴール方向との角度差: {angle_deg:.1f}°")

        if distance_m <= GOAL_THRESHOLD_M:
            print("\nゴール10m圏内に到達\n近距離フェーズへ移行")
            return 4

        print("ゴール方向へ旋回")
        turn_by_angle(bno=bno, angle_deg=angle_deg, motor_ok=motor_ok)

        print("5秒前進")
        stacked = forward_with_stack_check(
            bno=bno,
            duration=FORWARD_TIME,
            accel_threshold=stack_accel_threshold,
            motor_ok=motor_ok
        )
        if stacked:
            print("\nスタック検知")
            recovery_lat, recovery_lon = recover_from_stuck(bno=bno, motor_ok=motor_ok)

            if recovery_lat is not None and recovery_lon is not None:
                prev_lat = recovery_lat
                prev_lon = recovery_lon
            else:
                prev_lat = curr_lat
                prev_lon = curr_lon

            continue

        prev_lat = curr_lat
        prev_lon = curr_lon
        time.sleep(0.1)


# ============================================================
# メイン処理（phase1&2.py / phase3.py / phase4.py を1本に接続）
# ============================================================
def main():
    global GOAL_LAT, GOAL_LON  # phase4ブロック内でのGOAL_LAT/LON再代入をmodule変数に反映するため

    bno, cam, bme, qnh, motor_ok, gpio_ok = setup_sensors()

    phase = 1

    try:
        while True:

            # ==========================
            # 待機フェーズ（phase1&2.py そのまま）
            # ==========================
            if phase == 1:
                try:
                    if not bme:
                        phase = 2
                        continue

                    _, p, _ = bme.read_all()
                    if p is None:
                        time.sleep(0.5)
                        continue

                    alt = bme.altitude(p, qnh=qnh)
                    if alt is None:
                        time.sleep(0.5)
                        continue

                    print(f"[待機] alt={alt:.3f} m")
                    log_msg('alt', alt)
                    log_msg('msg', f"[待機] alt={alt:.3f} m")

                    if alt >= 10.0:
                        print("Go to falling phase")
                        log_msg('msg', 'Go to falling phase')
                        phase = 2
                    else:
                        time.sleep(1.0)

                except Exception as e:
                    print(f"Error in wait phase: {e}")
                    log_msg('error', f"Error in wait phase: {e}")
                    time.sleep(1)

            # ==========================
            # 落下フェーズ（phase1&2.py そのまま）
            # ==========================
            elif phase == 2:
                try:
                    # ① bme / gpio_ok のガード：continueではなくphase移行してbreakしない
                    if not bme:
                        print("BME280が使えないため落下フェーズをスキップします")
                        log_msg('warning', 'BME280 not available, skipping fall phase')
                        phase = 3
                        continue
                    if not gpio_ok:
                        print("GPIOが使えないためニクロム線を安全に駆動できません")
                        log_msg('warning', 'GPIO not available, cannot drive nichrome')
                        phase = 3
                        continue

                    FALL_TIMEOUT_SEC = 1800.0
                    fall_start_time = time.time()

                    consecutive_count = 0
                    REQUIRED_COUNT = 5  # 1秒ごとに計測し5回連続（=5秒間）で着地判定
                    D_ALT_THRESH = 0.5  # 仕様：5秒間の高度変化が0.1m以下

                    _, p, _ = bme.read_all()
                    if p is None:
                        print("初期高度の取得に失敗しました。再試行します。")
                        log_msg('warning', '初期高度の取得に失敗')
                        time.sleep(0.5)
                        continue  # phase==2のままwhile Trueの先頭へ戻り再試行

                    alt_prev = bme.altitude(p, qnh=qnh)
                    if alt_prev is None:
                        print("初期高度の計算に失敗しました。再試行します。")
                        log_msg('warning', '初期高度の計算に失敗')
                        time.sleep(0.5)
                        continue  # 同上

                    print(f"fall start alt={alt_prev:.3f} m")
                    log_msg('alt', alt_prev)
                    log_msg('msg', f"fall start alt={alt_prev:.3f} m")

                    while True:

                        # ② タイムアウトチェックをループ先頭で必ず実行
                        #    （Noneが続いてもタイムアウトで必ず抜けられる）
                        if time.time() - fall_start_time >= FALL_TIMEOUT_SEC:
                            print("300分経過 → 強制分離")
                            log_msg('warning', 'FALL timeout reached, forced separation')
                            break

                        time.sleep(1.0)

                        _, p, _ = bme.read_all()
                        if p is None:
                            print("BME280: read_all が None でした。スキップします。")
                            log_msg('warning', 'BME280 read_all returned None')
                            continue  # タイムアウトチェックは次ループで実行される

                        alt_now = bme.altitude(p, qnh=qnh)
                        if alt_now is None:
                            print("BME280: altitude が None でした。スキップします。")
                            log_msg('warning', 'BME280 altitude returned None')
                            continue  # 同上

                        d_alt = abs(alt_now - alt_prev)

                        print(
                            f"alt={alt_now:.3f} m, "
                            f"Δalt(1s)={d_alt:.3f} m "
                            f"({consecutive_count}/{REQUIRED_COUNT})"
                        )
                        log_msg('alt', alt_now)
                        log_msg('msg', f"Δalt(1s)={d_alt:.3f} m ({consecutive_count}/{REQUIRED_COUNT})")

                        # フェーズ移行の高度を7.5 mに
                        if alt_now <= 7.5 and d_alt <= D_ALT_THRESH:
                            consecutive_count += 1
                        else:
                            consecutive_count = 0

                        if consecutive_count >= REQUIRED_COUNT:
                            print("Landing detected")
                            log_msg('msg', 'Landing detected')
                            break

                        alt_prev = alt_now

                    # ニクロム線作動（パラシュート分離）
                    print("start nichrome wire")
                    log_msg('msg', 'start nichrome wire')
                    GPIO.output(NICHROME_PIN, 1)
                    time.sleep(15)
                    GPIO.output(NICHROME_PIN, 0)
                    print("finish nichrome wire")
                    log_msg('msg', 'finish nichrome wire')

                    phase = 3

                except Exception as e:
                    print(f"Error in falling phase: {e}")
                    log_msg('error', f"Error in falling phase: {e}")
                    time.sleep(1)

            # ==========================
            # 遠距離フェーズ（phase3.py の main() が
            # `phase = run_long_distance_phase(...)` としていたのと同じ呼び方）
            # ==========================
            elif phase == 3:
                phase = run_long_distance_phase(
                    bno=bno,
                    goal_lat=GOAL_LAT,
                    goal_lon=GOAL_LON,
                    stack_accel_threshold=STACK_ACCEL_THRESHOLD,
                    motor_ok=motor_ok
                )

            # ==========================
            # 近距離フェーズ（phase4.py の "if phase == 4:" ブロックそのまま）
            # ==========================
            elif phase == 4:
                # --- phase4.py の main() ゴール座標 ---
                GOAL_LAT = 40.14262816666667 # 本番
                GOAL_LON = 139.987715

                #ここに近距離フェーズの処理
                is_stacked = False
                print("\n--- フェーズ4: 近距離フェーズ（カメラ誘導） ---")
                if not cam:
                    print("カメラが認識されていません。フェーズ4をスキップします。")
                else:
                    is_inverted = False
                    lost_count = 0 #ターゲットを見失った連続回数をカウントする変数
                    
                    while phase == 4:
                        try:
                            #裏返り判定
                            if bno:
                                gravity = bno.gravity()
                                is_inverted = (gravity is not None and gravity[2] < -2.0)

                            #カメラで画像取得＆推論
                            cap = cam.capture_image()
                            if cap is None:
                                time.sleep(0.1)
                                continue
                            cap = cam.histogram_equalization(cap)
                            cap = cam.detect_cone(cap)
                            cx, cy, _, order = cam.get_cone_position(cap)

                            #orderに基づく行動
                            if order == 4:
                                print(f"ターゲットに超接近。ゴールと判定します！")
                                if motor_ok:
                                    md.stop()
                                break 
                                
                            elif order == 0:
                                print("ターゲットを見失いました。探索のため右回転します。")
                                lost_count += 1
                                if motor_ok:
                                    md.move('e', power=0.7, duration=0.1, is_inverted=is_inverted, enable_stack_check=False)
                                    
                                #10回連続（約5秒間）見失ったら、GPSで現在地を確認する
                                if lost_count >= 30:
                                    print("長時間ターゲットが見つかりません。現在地をGPSで確認します...")
                                    if motor_ok:
                                       md.stop()
                                                            
                                    curr_lat, curr_lon = idokeido()
                                    if curr_lat is not None and curr_lon is not None:
                                        d, _ = calculate_distance_and_angle(
                                            curr_lat, curr_lon, curr_lat, curr_lon, GOAL_LAT, GOAL_LON
                                        )
                                        print(f"ゴールまでの距離: {d:.2f}m")
                                                            
                                        if d <= 10.0:
                                            print("10m圏内を維持しています。カウントをリセットし、探索を継続します。")
                                            lost_count = 0 # まだ近くにいるので、もう一度探してみる
                                        else:
                                            print("10m圏外に出てしまいました。遠距離フェーズ(3)に戻ります。")
                                            phase = 3
                                            break
                                    else:
                                        print("GPS取得失敗。安全のため探索を継続します。")
                                        lost_count = 0 # 取得できなかった場合はとりあえず探索継続  
                                    
                            elif order == 1:
                                print("ターゲットは正面です。直進します。")
                                if motor_ok:
                                    is_stacked = md.move('w', power=0.7, duration=1.5, is_inverted=is_inverted, enable_stack_check=True)
                                    
                            elif order == 2:
                                print("ターゲットが右です。右に旋回してから前進します。")
                                if motor_ok:
                                    md.move('e', power=0.7, duration=0.1, is_inverted=is_inverted, enable_stack_check=False)                                        
                                    is_stacked = md.move('w', power=0.7, duration=1.5, is_inverted=is_inverted, enable_stack_check=True)
                            elif order == 3:
                                print("ターゲットが左です。左に旋回してから前進します。")
                                if motor_ok:
                                    md.move('q', power=0.7, duration=0.1, is_inverted=is_inverted, enable_stack_check=False) 
                                    is_stacked = md.move('w', power=0.7, duration=1.5, is_inverted=is_inverted, enable_stack_check=True)
                            # ④ スタック判定とリカバリー（motordriveにお任せ）
                            if motor_ok and is_stacked:
                                print("スタックを検知しました。リカバリー行動を開始します。")
                                md.check_stuck(is_stacked, is_inverted=is_inverted)
                                
                            time.sleep(0.1)

                        except Exception as e:
                            # ＝＝＝ ここからが追加したGPS安全装置 ＝＝＝
                            print(f"カメラ等でエラー発生: {e}")
                            if motor_ok:
                                md.stop() # 暴走防止のため一旦停止

                            print("GPSで現在地を確認し、10m圏内かチェックします。")
                            curr_lat, curr_lon = idokeido()

                            if curr_lat is not None and curr_lon is not None:
                                # 距離を計算（方位計算用の過去座標は不要なので現在地をダミーで入れています）
                                d, _ = calculate_distance_and_angle(
                                    curr_lat, curr_lon, curr_lat, curr_lon, GOAL_LAT, GOAL_LON
                                )
                                print(f"ゴールまでの距離: {d:.2f}m")

                                if d <= 10.0:
                                    print("10m圏内を維持しています。近距離フェーズを継続します。")
                                    time.sleep(0.1)
                                    continue # ループの先頭に戻ってカメラ再取得
                                else:
                                    print("10m圏外に出てしまいました。遠距離フェーズ(3)に戻ります。")
                                    phase = 3
                                    break # 近距離のループを抜けて、フェーズ3へ戻る
                            else:
                                print("GPSの取得にも失敗しました。安全のため近距離フェーズを維持してリトライします。")
                                time.sleep(0.1)
                                continue
                
                phase = 5

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n中断されました。")
        log_msg('msg', 'Interrupted by KeyboardInterrupt')

    finally:
        print("\n終了処理中...")
        log_msg('msg', 'Shutting down...')

        if cam:
            try:
                cam.close()
            except:
                pass
            try:
                cam.release()
            except:
                pass

        if bno:
            try:
                bno.close()
            except:
                pass

        if bme:
            try:
                bme.close()
            except:
                pass

        if motor_ok:
            try:
                md.cleanup()
            except:
                pass

        if gpio_ok:
            try:
                GPIO.output(LED_PIN, 0)
                GPIO.output(NICHROME_PIN, 0)
                GPIO.cleanup()
            except:
                pass

        try:
            cv2.destroyAllWindows()
        except:
            pass

        print("完了。お疲れ様でした。")
        log_msg('msg', 'Done. Good job.')


if __name__ == "__main__":
    main()
