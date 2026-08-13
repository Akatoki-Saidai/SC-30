# ============================================================
# EM.py  ---  CanSat 統合メインプログラム (SC-30)
#
#   phase1 : 待機フェーズ       (BME280 高度監視)
#   phase2 : 落下フェーズ       (着地検知 → ニクロム線でパラシュート分離)
#   phase3 : 遠距離フェーズ     (GPS + BNO055 誘導)
#   phase4 : 近距離フェーズ     (カメラ誘導)
#   phase5 : ゴール / 終了
#
# 元ファイル: phase1&2.py / phase3.py / phase4.py を統合。
#
# 【統合にあたっての方針】
#   ・センサ初期化と終了処理は「全体で1回だけ」
#   ・各フェーズは「次のフェーズ番号を return する関数」
#   ・設定値はすべてこのファイル上部に集約
#   ・呼び出しは現在の各モジュールの実APIに合わせてある
#     (camera_sc30.py の capture_image/detect_cone/get_cone_position /
#      motordrive.py の 5引数 move など)
# ============================================================

import time
import math
import threading

# ------------------------------------------------------------
# 外部ライブラリ (実機以外でも起動できるよう個別にガード)
# ------------------------------------------------------------
try:
    import cv2
except Exception as e:
    cv2 = None
    print(f"cv2 import error: {e}")

try:
    import RPi.GPIO as GPIO
except Exception as e:
    GPIO = None
    print(f"RPi.GPIO import error: {e}")


# ============================================================
# ★★★ 設定 ★★★
# ============================================================

# ------------------------------------------------------------
# 開始フェーズ (単体テスト時は 3 や 4 にすれば途中から実行できる)
# ------------------------------------------------------------
START_PHASE = 1

# ------------------------------------------------------------
# ゴール座標
#   ★元の3ファイルで座標が食い違っていたので1か所に統一。
#   ★本番当日は「機体に載せるGPSモジュール自身」でコーン位置を
#     60秒以上ログして平均した値に差し替えること（共通誤差が相殺される）。
# ------------------------------------------------------------
GOAL_LAT = 40.14266434          # 能代宇宙広場 (昨年度本番値)
GOAL_LON = 139.9876452

# GOAL_LAT, GOAL_LON = 40.1426151,  139.9876656   # phase3.py 記載値
# GOAL_LAT, GOAL_LON = 40.21625,    140.02270     # phase3.py テスト用
# GOAL_LAT, GOAL_LON = 40.212932,   140.018288    # phase4.py 能代公園

# ------------------------------------------------------------
# ピン設定 (BCM)
#   ※ LED_PIN は motordrive.PIN_LED と同じ 5 番。
#     motordrive.setup_gpio() でも設定されるが競合はしない。
# ------------------------------------------------------------
LED_PIN = 5
NICHROME_PIN = 16

# ------------------------------------------------------------
# カメラ
#   camera_sc30.py の Camera を使う (引数なしで初期化 → start() が必要)。
#   API: capture_image() -> ndarray
#        histogram_equalization(image) -> image
#        detect_cone(image) -> mask
#        get_cone_position(image) -> (cx, cy, image, camera_order)
# ------------------------------------------------------------
SHOW_CAMERA_WINDOW = False      # 実機ヘッドレス運用では False

# ------------------------------------------------------------
# phase1 待機フェーズ
# ------------------------------------------------------------
WAIT_ALT_THRESHOLD = 10.0       # この高度[m]を超えたら落下フェーズへ
WAIT_INTERVAL = 1.0

# ------------------------------------------------------------
# phase2 落下フェーズ
# ------------------------------------------------------------
LANDING_ALT_THRESHOLD = 7.5     # 着地判定に入る高度[m]
D_ALT_THRESH = 0.5              # 1秒あたりの高度変化がこれ以下なら静止とみなす
REQUIRED_COUNT = 5              # 上記が5回連続(=5秒)で着地確定
FALL_TIMEOUT_SEC = 1800.0       # 保険: この時間で強制分離
NICHROME_ON_SEC = 15.0          # ニクロム線の通電時間

# ------------------------------------------------------------
# phase3 遠距離フェーズ
# ------------------------------------------------------------
POWER = 0.7                     # ※motordrive内部で MAX_POWER_LIMIT(≒0.71) が更に掛かる
INITIAL_FORWARD_TIME = 5.0      # サブキャリア離脱の前進
FORWARD_TIME = 20.0             # 通常の1回あたり前進時間
GOAL_THRESHOLD_M = 10.0         # この距離まで近づいたら近距離フェーズへ

OMEGA_DEG_PER_SEC = 180.0       # 旋回角速度の想定値 ★要実測
TURN_TOLERANCE_DEG = 15.0       # この角度差以内なら旋回しない
MIN_TURN_TIME = 0.3
MAX_TURN_TIME = 4.0
MAX_TURN_ATTEMPTS = 3           # BNO055 によるヨー角補正の最大回数

GPS_FAIL_LIMIT = 6              # GPS 連続失敗がこの回数で近距離フェーズへ
PHASE3_TIMEOUT_SEC = 1800.0     # 遠距離フェーズの上限時間 (保険)
INIT_POSTURE_TIMEOUT_SEC = 120.0  # 初期姿勢確認ループの上限時間

# --- スタック検知の方式選択 ---------------------------------
#   True : motordrive.move(enable_stack_check=True) の内蔵検知を使う
#          (戻り値 1=スタック / 2=反転、復帰は motordrive.check_stuck に任せる)
#   False: phase3.py 由来の自前スレッド検知を使う
#          (linear_acceleration が閾値以下の状態が継続したらスタック)
USE_MOTORDRIVE_STACK_CHECK = True

# 自前検知を使う場合のパラメータ (USE_MOTORDRIVE_STACK_CHECK = False のとき有効)
STACK_ACCEL_THRESHOLD = 0.5     # ★実験によって決める必要がある [m/s^2]
STACK_HOLD_TIME = 1.0
STACK_SAMPLE_TIME = 0.05

# 自前スタック復帰 (phase3.py 由来)
BACK_TIME = 3.0                 # 後退
RECOVERY_TURN_DEG = 60.0        # 右旋回角
RECOVERY_FORWARD_TIME = 2.0     # 前進

# ------------------------------------------------------------
# phase4 近距離フェーズ
# ------------------------------------------------------------
CAM_TURN_TIME = 0.1             # コーンを画角に入れるための微小旋回
CAM_FORWARD_TIME = 1.5          # コーン方向への前進 (motordrive の検知条件 >=1.5)
LOST_LIMIT = 30                 # この回数連続で見失ったら GPS 確認
NEAR_RADIUS_M = 10.0            # この距離を超えたら遠距離フェーズへ戻る
PHASE4_TIMEOUT_SEC = 900.0      # 近距離フェーズの上限時間 (保険)

# phase4 → phase3 の往復を制限する
MAX_RETURN_TO_PHASE3 = 3

# ------------------------------------------------------------
# 旋回コマンド
#   motordrive.py の実装:
#     'w' 前進 / 's' 後退
#     'a' 左旋回(片輪減速) / 'd' 右旋回(片輪減速)
#     'q' その場左 (mr=+p, ml=-p) / 'e' その場右 (mr=-p, ml=+p)
#   ★phase3.py は LEFT='e', RIGHT='q' としていたが現行実装では逆。ここで訂正。
# ------------------------------------------------------------
LEFT_TURN_CMD = "q"
RIGHT_TURN_CMD = "e"

# ------------------------------------------------------------
# 機体反転判定
#   motordrive.py の内部判定: accel[2] <= -2.0 なら反転
#   → 正常姿勢では z 軸は「正」。phase4.py の gravity[2] < -2.0 と同じ規約。
#   （phase3.py の「gravity[2] < 0 なら正常」は誤りだったのでこちらに統一）
# ------------------------------------------------------------
GRAVITY_Z_INVERTED_THRESHOLD = -2.0


# ============================================================
# モジュール読み込み (個別ガード)
# ============================================================
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
    from camera_sc30 import Camera
except Exception as e:
    Camera = None
    print(f"Camera import error: {e}")

try:
    from gps import idokeido, calculate_distance_and_angle, ERROR_DISTANCE
except Exception as e:
    idokeido = None
    calculate_distance_and_angle = None
    ERROR_DISTANCE = 2727272727
    print(f"GPS import error: {e}")

try:
    import motordrive as md
except Exception as e:
    md = None
    print(f"motordrive import error: {e}")

try:
    import make_csv
except Exception as e:
    make_csv = None
    print(f"make_csv import error: {e}")

try:
    import ijochi
except Exception as e:
    ijochi = None
    print(f"ijochi import error: {e}")


# ============================================================
# ログ
# ============================================================
def log_msg(msg_type, msg_data):
    """make_csv によるログ保存。失敗しても主処理は止めない。"""
    try:
        if make_csv is not None:
            make_csv.print(msg_type, msg_data)
    except Exception:
        pass


def say(msg, msg_type="msg"):
    """画面出力と CSV ログを同時に行う。"""
    print(msg)
    log_msg(msg_type, msg)


def filt(sensor_name, value_name, value):
    """ijochi による異常値棄却。ijochi が無ければ素通し。"""
    if ijochi is None:
        return value
    try:
        return ijochi.abnormal_check(sensor_name, value_name, value, ERROR_FLAG=False)
    except Exception:
        return value


# ============================================================
# 機体状態をまとめて持ち回すコンテナ
# ============================================================
class Context:
    def __init__(self):
        self.bno = None
        self.cam = None
        self.bme = None
        self.qnh = 1013.25
        self.motor_ok = False
        self.gpio_ok = False

        # phase4 から phase3 へ戻った回数
        self.return_to_phase3_count = 0
        self.allow_return_to_phase3 = True


# ============================================================
# セットアップ
# ============================================================
def setup_devices():
    ctx = Context()

    log_msg("goal_lat", GOAL_LAT)
    log_msg("goal_lon", GOAL_LON)

    # --- BNO055 ---
    say("bnoセットアップ開始")
    try:
        if BNO055 is not None:
            bno = BNO055()
            if bno.begin():
                ctx.bno = bno
                say("BNO055 初期化成功")
            else:
                say("BNO055: Init Failed", "error")
        else:
            say("BNO055 モジュールが利用できません（インポート失敗）", "error")
    except Exception as e:
        say(f"BNO055 Setup Error: {e}", "error")

    # --- Camera ---
    say("cameraセットアップ開始")
    try:
        if Camera is not None:
            cam = Camera()
            cam.start()
            ctx.cam = cam
            say("Camera 初期化完了")
        else:
            say("Camera モジュールが利用できません（インポート失敗）", "error")
    except Exception as e:
        say(f"Camera Setup Error: {e}", "error")
        ctx.cam = None

    # --- BME280 ---
    say("bmeセットアップ開始")
    try:
        if BME280Sensor is not None:
            bme = BME280Sensor(debug=False)
            if bme.calib_ok:
                ctx.bme = bme
                # 起動直後は値が安定しないため空読みしてから基準気圧を取る
                for _ in range(20):
                    try:
                        bme.read_all()
                    except Exception:
                        pass
                    time.sleep(0.02)

                _, p0, _ = bme.read_all()
                if p0 is not None:
                    log_msg("alt_base_press", p0)

                ctx.qnh = bme.baseline()
                say(f"BME280 baseline qnh={ctx.qnh}")
            else:
                say("BME280: Calibration Failed", "error")
        else:
            say("BME280 モジュールが利用できません（インポート失敗）", "error")
    except Exception as e:
        say(f"BME280 Setup Error: {e}", "error")

    # --- Motor ---
    say("モータセットアップ開始")
    try:
        if md is not None:
            md.setup_motors()
            ctx.motor_ok = True
            say("モーター初期化成功")
        else:
            say("motordrive モジュールが利用できません（インポート失敗）", "error")
    except Exception as e:
        say(f"Motor Setup Error: {e}", "error")

    # --- GPIO (LED, ニクロム線) ---
    say("GPIOセットアップ開始")
    try:
        if GPIO is not None:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(LED_PIN, GPIO.OUT)
            GPIO.setup(NICHROME_PIN, GPIO.OUT)
            # 【超重要】起動直後は必ず OFF にする（ニクロム線の誤爆防止）
            GPIO.output(LED_PIN, 0)
            GPIO.output(NICHROME_PIN, 0)
            ctx.gpio_ok = True
        else:
            say("RPi.GPIO が利用できません（インポート失敗）", "error")
    except Exception as e:
        say(f"GPIO Setup Error: {e}", "error")

    print("\n=== デバイス接続状況 ===")
    print(f"* BNO055 : {'OK' if ctx.bno else 'Skip'}")
    print(f"* Camera : {'OK' if ctx.cam else 'Skip'}")
    print(f"* BME280 : {'OK' if ctx.bme else 'Skip'}")
    print(f"* Motors : {'OK' if ctx.motor_ok else 'Skip'}")
    print(f"* GPIO   : {'OK' if ctx.gpio_ok else 'Skip'}")
    print("========================\n")

    say("Finished setup")
    return ctx


# ============================================================
# 共通ヘルパー
# ============================================================
def normalize_angle(angle_deg):
    """角度を -180 ～ +180° に変換"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def read_altitude(ctx):
    """BME280 から高度[m]を取得。失敗したら None。"""
    if ctx.bme is None:
        return None
    try:
        t, p, _ = ctx.bme.read_all()
        p = filt("bme", "pressure", p)
        if p is None:
            return None

        t = filt("bme", "temperature", t)
        if t is not None:
            log_msg("temp", t)
        log_msg("press", p)

        return ctx.bme.altitude(p, qnh=ctx.qnh)
    except Exception as e:
        say(f"BME280 read error: {e}", "error")
        return None


def read_gps():
    """GPS から (lat, lon) を取得。失敗したら (None, None)。"""
    if idokeido is None:
        return None, None
    try:
        lat, lon = idokeido()
    except Exception as e:
        say(f"GPS read error: {e}", "error")
        return None, None

    if lat is None or lon is None:
        return None, None

    lat = filt("gps", "latitude", lat)
    lon = filt("gps", "longitude", lon)
    if lat is None or lon is None:
        return None, None

    log_msg("lat_lon", (lat, lon))
    return lat, lon


def distance_and_angle(curr_lat, curr_lon, prev_lat, prev_lon):
    """
    ゴールまでの (距離[m], 角度[rad]) を返す。失敗したら (None, None)。
    角度は gps.py の規約で 正=左 / 負=右。
    """
    if calculate_distance_and_angle is None:
        return None, None
    try:
        d, a = calculate_distance_and_angle(
            curr_lat, curr_lon,
            prev_lat, prev_lon,
            GOAL_LAT, GOAL_LON
        )
    except Exception as e:
        say(f"distance calc error: {e}", "error")
        return None, None

    if d is None or d == ERROR_DISTANCE:
        return None, None
    return d, a


def is_inverted(ctx):
    """
    機体が反転しているかを判定する。
    True=反転 / False=正常 / None=判定不能
    """
    if ctx.bno is None:
        return None
    try:
        g = ctx.bno.gravity()
    except Exception as e:
        say(f"gravity read error: {e}", "error")
        return None

    g = filt("bno", "gravity", g)
    if g is None:
        return None

    log_msg("grav", g)
    return g[2] < GRAVITY_Z_INVERTED_THRESHOLD


def blink_led(ctx, times=3, interval=0.5):
    if not ctx.gpio_ok or GPIO is None:
        return
    try:
        for _ in range(times):
            GPIO.output(LED_PIN, 1)
            time.sleep(interval)
            GPIO.output(LED_PIN, 0)
            time.sleep(interval)
    except Exception:
        pass


def motor_move(ctx, cmd, power, duration, inverted=False, stack_check=False):
    """
    motordrive.move の薄いラッパ。
    戻り値: 0=正常 / 1=スタック / 2=反転 (モーター無効時も 0)
    """
    if not ctx.motor_ok or md is None:
        return 0
    try:
        ret = md.move(
            cmd,
            power=power,
            duration=duration,
            is_inverted=bool(inverted),
            enable_stack_check=bool(stack_check)
        )
        # ★motor_l/motor_r列は左右モーター出力値用のため、
        #   コマンド文字と秒数はmsgとして残す(motorキーの誤用を修正)
        log_msg("msg", f"motor cmd={cmd} power={power} duration={duration}")
        return ret if isinstance(ret, int) else 0
    except Exception as e:
        # move() 内部で例外が出るとモーターが回りっぱなしになるため必ず止める
        say(f"motor move error: {e}", "error")
        motor_stop(ctx)
        return 0


def motor_stop(ctx):
    if not ctx.motor_ok or md is None:
        return
    try:
        md.stop()
    except Exception as e:
        say(f"motor stop error: {e}", "error")


def handle_stack_result(ctx, ret, inverted=False):
    """
    motordrive.move の戻り値に応じて復帰動作を行う。
    戻り値: True=何らかの復帰動作を実行した
    """
    if not ctx.motor_ok or md is None:
        return False
    if ret not in (1, 2):
        return False

    if ret == 1:
        say("スタックを検知しました。リカバリー行動を開始します。", "warning")
    else:
        say("反転を検知しました。復帰行動を開始します。", "warning")

    try:
        md.check_stuck(ret, is_inverted=inverted)
    except Exception as e:
        say(f"check_stuck error: {e}", "error")
        motor_stop(ctx)
    return True


# ============================================================
# phase1 : 待機フェーズ
# ============================================================
def run_phase1(ctx):
    say("\n========== phase1 : 待機フェーズ ==========")
    log_msg("phase", 1)

    if ctx.bme is None:
        say("BME280 が使えないため待機フェーズをスキップします", "warning")
        return 2

    while True:
        try:
            alt = read_altitude(ctx)
            if alt is None:
                time.sleep(0.5)
                continue

            print(f"[待機] alt={alt:.3f} m")
            log_msg("alt", alt)

            if alt >= WAIT_ALT_THRESHOLD:
                say("Go to falling phase")
                return 2

            time.sleep(WAIT_INTERVAL)

        except KeyboardInterrupt:
            raise
        except Exception as e:
            say(f"Error in wait phase: {e}", "error")
            time.sleep(1.0)


# ============================================================
# phase2 : 落下フェーズ
# ============================================================
def fire_nichrome(ctx):
    """ニクロム線を通電してパラシュートを分離する。"""
    if not ctx.gpio_ok or GPIO is None:
        say("GPIO が使えないためニクロム線を駆動できません", "warning")
        return

    say("start nichrome wire")
    try:
        GPIO.output(NICHROME_PIN, 1)
        time.sleep(NICHROME_ON_SEC)
    finally:
        # 例外や Ctrl-C でも必ず OFF に戻す（焼き切りっぱなし防止）
        try:
            GPIO.output(NICHROME_PIN, 0)
        except Exception:
            pass
    say("finish nichrome wire")


def run_phase2(ctx):
    say("\n========== phase2 : 落下フェーズ ==========")
    log_msg("phase", 2)

    if ctx.bme is None:
        say("BME280 が使えないため落下フェーズをスキップします", "warning")
        return 3

    fall_start_time = time.monotonic()
    consecutive_count = 0
    alt_prev = None

    # 初期高度の取得（取れるまで待つ。ただし全体タイムアウトは効く）
    while alt_prev is None:
        if time.monotonic() - fall_start_time >= FALL_TIMEOUT_SEC:
            say("落下フェーズがタイムアウトしました → 強制分離", "warning")
            fire_nichrome(ctx)
            return 3

        alt_prev = read_altitude(ctx)
        if alt_prev is None:
            say("初期高度の取得に失敗しました。再試行します。", "warning")
            time.sleep(0.5)

    say(f"fall start alt={alt_prev:.3f} m")
    log_msg("alt", alt_prev)

    while True:
        try:
            # タイムアウト判定はループ先頭で必ず行う
            # (高度が None のまま続いても必ず抜けられる)
            if time.monotonic() - fall_start_time >= FALL_TIMEOUT_SEC:
                say("落下フェーズがタイムアウトしました → 強制分離", "warning")
                break

            time.sleep(1.0)

            alt_now = read_altitude(ctx)
            if alt_now is None:
                say("BME280: 高度取得に失敗。スキップします。", "warning")
                continue

            d_alt = abs(alt_now - alt_prev)

            print(
                f"alt={alt_now:.3f} m, "
                f"Δalt(1s)={d_alt:.3f} m "
                f"({consecutive_count}/{REQUIRED_COUNT})"
            )
            log_msg("alt", alt_now)

            if alt_now <= LANDING_ALT_THRESHOLD and d_alt <= D_ALT_THRESH:
                consecutive_count += 1
            else:
                consecutive_count = 0

            if consecutive_count >= REQUIRED_COUNT:
                say("Landing detected")
                break

            alt_prev = alt_now

        except KeyboardInterrupt:
            raise
        except Exception as e:
            say(f"Error in falling phase: {e}", "error")
            time.sleep(1.0)

    # パラシュート分離
    fire_nichrome(ctx)
    return 3


# ============================================================
# phase3 : 遠距離フェーズ (GPS 誘導)
# ============================================================
def correct_orientation(ctx):
    """
    機体が反転していた場合に正しい姿勢へ戻す動作。
    ★phase3.py では最後に NotImplementedError を投げていたが、
      ミッションを止めないよう「試して先へ進む」形にしてある。
    ★実機の機構に合わせて下記のもがき動作を調整すること。
    """
    say("機体の反転を検知 → 姿勢復帰シーケンス開始", "warning")
    blink_led(ctx, times=3, interval=0.5)

    try:
        # TODO: 実機に合わせて調整する
        motor_move(ctx, "w", 1.0, 10.0, inverted=True, stack_check=False)
        motor_stop(ctx)
    except Exception as e:
        say(f"Error in correct_orientation: {e}", "error")

    say("姿勢復帰シーケンス終了")


def turn_by_angle(ctx, angle_deg):
    """
    ゴール方向へ旋回する。
    gps.py の規約: angle > 0 → 左, angle < 0 → 右
    """
    if not ctx.motor_ok:
        say("モーターが使用できないため旋回できません", "warning")
        return

    if abs(angle_deg) <= TURN_TOLERANCE_DEG:
        print(f"方向差 {abs(angle_deg):.1f}° は許容範囲内のため旋回不要")
        return

    def _turn(deg):
        cmd = LEFT_TURN_CMD if deg > 0 else RIGHT_TURN_CMD
        t = abs(deg) / OMEGA_DEG_PER_SEC
        t = min(max(t, MIN_TURN_TIME), MAX_TURN_TIME)
        motor_move(ctx, cmd, POWER, t, inverted=False, stack_check=False)
        return t

    # --- BNO055 が無い場合: 時間だけで旋回する ---
    if ctx.bno is None:
        _turn(angle_deg)
        return

    # --- BNO055 フィードバック旋回 ---
    try:
        euler = ctx.bno.euler()
    except Exception as e:
        say(f"BNO055 euler error: {e}", "error")
        euler = None

    # ※ijochi の異常値テーブルに "euler" の項目は無いため filt は掛けない
    if euler is None:
        say("BNO055 Euler角取得失敗 → 時間指定で旋回します", "warning")
        _turn(angle_deg)
        return

    log_msg("euler", euler)
    start_yaw = euler[0]
    target_yaw = (start_yaw + angle_deg) % 360.0
    print(f"旋回開始：現在Yaw={start_yaw:.1f}° / 目標Yaw={target_yaw:.1f}°")

    current_yaw = start_yaw
    for attempt in range(MAX_TURN_ATTEMPTS):
        try:
            euler = ctx.bno.euler()
        except Exception:
            euler = None

        if euler is None:
            say("Yaw取得失敗", "warning")
            break

        current_yaw = euler[0]
        diff = normalize_angle(target_yaw - current_yaw)
        print(f"現在Yaw={current_yaw:.1f}° / 残り={diff:.1f}°")

        if abs(diff) <= TURN_TOLERANCE_DEG:
            print("旋回完了")
            return

        t = _turn(diff)
        print(f"旋回補正 {attempt + 1}/{MAX_TURN_ATTEMPTS} {t:.2f}s")
        time.sleep(0.3)

    print(f"旋回終了：目標Yaw={target_yaw:.1f}° / 現在Yaw={current_yaw:.1f}°")


def forward_with_own_stack_check(ctx, duration):
    """
    phase3.py 由来の自前スタック検知。
    モーターを別スレッドで回しつつ linear_acceleration を監視し、
    ax, ay, az がすべて閾値以下の状態が継続したらスタックと判定する。
    戻り値: True=スタック検知
    """
    if not ctx.motor_ok or md is None:
        say("モーターが使用できません", "warning")
        return False

    # BNO055 が無い場合は判定なしで前進 (phase3.py は例外にしていたが継続する)
    if ctx.bno is None:
        say("BNO055 が無いためスタック判定なしで前進します", "warning")
        motor_move(ctx, "w", POWER, duration)
        return False

    if STACK_ACCEL_THRESHOLD is None or STACK_ACCEL_THRESHOLD <= 0:
        raise ValueError("STACK_ACCEL_THRESHOLD は正の値に設定してください")

    motor_error = []

    def motor_task():
        try:
            md.move("w", power=POWER, duration=duration,
                    is_inverted=False, enable_stack_check=False)
        except Exception as e:
            motor_error.append(e)

    motor_thread = threading.Thread(target=motor_task, daemon=True)
    motor_thread.start()

    low_accel_start = None
    stacked = False

    while motor_thread.is_alive():
        try:
            accel = ctx.bno.linear_acceleration()
        except Exception:
            accel = None

        if accel is None:
            low_accel_start = None
            time.sleep(STACK_SAMPLE_TIME)
            continue

        ax, ay, az = accel
        low_accel = (
            abs(ax) <= STACK_ACCEL_THRESHOLD and
            abs(ay) <= STACK_ACCEL_THRESHOLD and
            abs(az) <= STACK_ACCEL_THRESHOLD
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
        say(f"モータースレッドでエラー: {motor_error[0]}", "error")
        motor_stop(ctx)

    return stacked


def recover_from_stuck_own(ctx):
    """phase3.py 由来のスタック復帰: 3秒後退 → 60°右旋回 → 2秒前進"""
    if not ctx.motor_ok:
        return None, None

    say("\n===== スタック復帰開始 =====", "warning")
    blink_led(ctx, times=2, interval=0.2)

    print("3秒後退")
    motor_move(ctx, "s", POWER, BACK_TIME)

    print(f"{RECOVERY_TURN_DEG:.0f}°右旋回")
    turn_by_angle(ctx, -RECOVERY_TURN_DEG)

    # 前進直前の座標を記録し、次回の方位推定の基準にする
    start_lat, start_lon = read_gps()

    print("2秒前進")
    motor_move(ctx, "w", POWER, RECOVERY_FORWARD_TIME)

    time.sleep(0.5)
    say("スタック復帰終了")
    return start_lat, start_lon


def run_phase3(ctx):
    say("\n========== phase3 : 遠距離フェーズ ==========")
    log_msg("phase", 3)

    phase_start = time.monotonic()

    # ------------------------------------------------------------
    # 初期姿勢確認ループ (GPS が取れて、姿勢が確認できるまで)
    # ------------------------------------------------------------
    curr_lat = curr_lon = None
    posture_start = time.monotonic()

    while True:
        if time.monotonic() - posture_start >= INIT_POSTURE_TIMEOUT_SEC:
            say("初期姿勢確認がタイムアウトしました → そのまま走行を開始します", "warning")
            break

        print("\n--- 初期センサ取得 ---")
        curr_lat, curr_lon = read_gps()

        # 参考情報としてログに残す (phase3.py のフローチャート準拠)
        if ctx.bno is not None:
            try:
                accel = filt("bno", "linear_accel", ctx.bno.linear_acceleration())
                gyro = filt("bno", "gyro", ctx.bno.gyroscope())
                mag = filt("bno", "mag", ctx.bno.magnetometer())
                if accel is not None:
                    log_msg("accel_line", accel)
                if gyro is not None:
                    log_msg("gyro", gyro)
                if mag is not None:
                    log_msg("mag", mag)
            except Exception as e:
                say(f"BNO055 read error: {e}", "error")

        if curr_lat is None or curr_lon is None:
            print("GPS取得失敗")
            time.sleep(1.0)
            continue

        print(f"GPS: {curr_lat}, {curr_lon}")

        inv = is_inverted(ctx)
        if inv is None:
            print("姿勢判定不能（重力加速度取得失敗）→ 正常とみなして続行")
            break
        if inv is False:
            print("機体は正常な向きです")
            break

        correct_orientation(ctx)
        break

    prev_lat = curr_lat
    prev_lon = curr_lon

    # ------------------------------------------------------------
    # サブキャリア離脱
    # ------------------------------------------------------------
    say(f"\nサブキャリア離脱のため{INITIAL_FORWARD_TIME:.0f}秒前進")
    motor_move(ctx, "w", POWER, INITIAL_FORWARD_TIME)
    motor_stop(ctx)
    time.sleep(1.0)

    # 前進後の位置を方位推定の基準にする
    if prev_lat is None or prev_lon is None:
        prev_lat, prev_lon = read_gps()

    gps_fail_count = 0

    # ------------------------------------------------------------
    # 遠距離メインループ
    # ------------------------------------------------------------
    while True:
        try:
            if time.monotonic() - phase_start >= PHASE3_TIMEOUT_SEC:
                say("遠距離フェーズがタイムアウトしました → 近距離フェーズへ", "warning")
                motor_stop(ctx)
                return 4

            print("\n--- 遠距離走行ループ ---")
            curr_lat, curr_lon = read_gps()

            if curr_lat is None or curr_lon is None:
                gps_fail_count += 1
                say(f"GPS取得失敗 {gps_fail_count}/{GPS_FAIL_LIMIT}", "warning")
                if gps_fail_count >= GPS_FAIL_LIMIT:
                    say("GPS取得失敗が連続 → 近距離フェーズへ移行")
                    motor_stop(ctx)
                    return 4
                time.sleep(1.0)
                continue

            gps_fail_count = 0
            print(f"GPS: {curr_lat}, {curr_lon}")

            if prev_lat is None or prev_lon is None:
                # 基準点がまだ無い場合は今回の座標を基準にして次周へ
                prev_lat, prev_lon = curr_lat, curr_lon
                say("方位推定の基準点を取得しました。次のループから走行します。", "warning")
                time.sleep(0.5)
                continue

            distance_m, angle_rad = distance_and_angle(
                curr_lat, curr_lon, prev_lat, prev_lon
            )
            if distance_m is None:
                say("距離・角度計算失敗", "warning")
                time.sleep(0.5)
                continue

            angle_deg = math.degrees(angle_rad)
            say(f"ゴールまでの距離: {distance_m:.2f} m / 角度差: {angle_deg:.1f}°")
            log_msg("goal_distance", distance_m)
            log_msg("goal_relative_angle_rad", angle_rad)

            if distance_m <= GOAL_THRESHOLD_M:
                say(f"\nゴール{GOAL_THRESHOLD_M:.0f}m圏内に到達 → 近距離フェーズへ移行")
                motor_stop(ctx)
                return 4

            print("ゴール方向へ旋回")
            turn_by_angle(ctx, angle_deg)

            print(f"{FORWARD_TIME:.0f}秒前進")

            if USE_MOTORDRIVE_STACK_CHECK:
                # motordrive 内蔵のスタック / 反転検知を使う
                inv = is_inverted(ctx)
                inverted = bool(inv) if inv is not None else False
                ret = motor_move(ctx, "w", POWER, FORWARD_TIME,
                                 inverted=inverted, stack_check=True)
                if handle_stack_result(ctx, ret, inverted=inverted):
                    # 復帰動作で機体が動いたので、基準点を取り直す
                    prev_lat, prev_lon = read_gps()
                    if prev_lat is None:
                        prev_lat, prev_lon = curr_lat, curr_lon
                    continue
            else:
                # phase3.py 由来の自前スタック検知を使う
                if forward_with_own_stack_check(ctx, FORWARD_TIME):
                    say("\nスタック検知", "warning")
                    rec_lat, rec_lon = recover_from_stuck_own(ctx)
                    if rec_lat is not None and rec_lon is not None:
                        prev_lat, prev_lon = rec_lat, rec_lon
                    else:
                        prev_lat, prev_lon = curr_lat, curr_lon
                    continue

            prev_lat, prev_lon = curr_lat, curr_lon
            time.sleep(0.1)

        except KeyboardInterrupt:
            raise
        except Exception as e:
            say(f"Error in long distance phase: {e}", "error")
            motor_stop(ctx)
            time.sleep(1.0)


# ============================================================
# phase4 : 近距離フェーズ (カメラ誘導)
# ============================================================
def check_near_goal(ctx):
    """
    GPS でゴールから NEAR_RADIUS_M 以内にいるかを確認する。
    戻り値: True=圏内 / False=圏外 / None=取得失敗
    """
    curr_lat, curr_lon = read_gps()
    if curr_lat is None or curr_lon is None:
        return None

    # 方位は使わないので基準点には現在地を入れる
    d, _ = distance_and_angle(curr_lat, curr_lon, curr_lat, curr_lon)
    if d is None:
        return None

    say(f"ゴールまでの距離: {d:.2f} m")
    log_msg("goal_distance", d)
    return d <= NEAR_RADIUS_M


def run_phase4(ctx):
    say("\n========== phase4 : 近距離フェーズ（カメラ誘導） ==========")
    log_msg("phase", 4)

    if ctx.cam is None:
        say("カメラが認識されていません。近距離フェーズをスキップします。", "warning")
        return 5

    phase_start = time.monotonic()
    lost_count = 0

    while True:
        try:
            if time.monotonic() - phase_start >= PHASE4_TIMEOUT_SEC:
                say("近距離フェーズがタイムアウトしました → 終了フェーズへ", "warning")
                motor_stop(ctx)
                return 5

            # --- 反転判定 ---
            inv = is_inverted(ctx)
            inverted = bool(inv) if inv is not None else False

            # --- カメラで画像取得 & 推論 ---
            # camera_sc30.py の API:
            #   capture_image() -> histogram_equalization() -> detect_cone()
            #   -> get_cone_position() -> (cx, cy, image, camera_order)
            frame = ctx.cam.capture_image()
            if frame is None:
                time.sleep(0.1)
                continue

            eq = ctx.cam.histogram_equalization(frame)
            mask = ctx.cam.detect_cone(eq)
            cx, cy, mask_img, order = ctx.cam.get_cone_position(mask)

            log_msg("camera_order", order)
            log_msg("camera_center_x", cx)
            log_msg("camera_center_y", cy)

            if SHOW_CAMERA_WINDOW and cv2 is not None and mask_img is not None:
                try:
                    cv2.imshow("kekka", mask_img)
                    cv2.waitKey(1)
                except Exception:
                    pass

            ret = 0

            # --- order に基づく行動 ---
            if order == 4:
                say("ターゲットに超接近。ゴールと判定します！")
                motor_stop(ctx)
                return 5

            elif order == 0:
                lost_count += 1
                print(f"ターゲットを見失いました（{lost_count}/{LOST_LIMIT}）。探索のため右回転します。")
                motor_move(ctx, RIGHT_TURN_CMD, POWER, CAM_TURN_TIME,
                           inverted=inverted, stack_check=False)

                if lost_count >= LOST_LIMIT:
                    say("長時間ターゲットが見つかりません。現在地をGPSで確認します。", "warning")
                    motor_stop(ctx)

                    near = check_near_goal(ctx)
                    if near is True:
                        print(f"{NEAR_RADIUS_M:.0f}m圏内を維持しています。探索を継続します。")
                        lost_count = 0
                    elif near is False:
                        if ctx.allow_return_to_phase3:
                            say(f"{NEAR_RADIUS_M:.0f}m圏外に出ました → 遠距離フェーズへ戻ります", "warning")
                            return 3
                        say("往復回数の上限に達したため近距離フェーズを継続します", "warning")
                        lost_count = 0
                    else:
                        say("GPS取得失敗。安全のため探索を継続します。", "warning")
                        lost_count = 0

            elif order == 1:
                lost_count = 0
                print("ターゲットは正面です。直進します。")
                ret = motor_move(ctx, "w", POWER, CAM_FORWARD_TIME,
                                 inverted=inverted, stack_check=True)

            elif order == 2:
                lost_count = 0
                print("ターゲットが右です。右に旋回してから前進します。")
                motor_move(ctx, RIGHT_TURN_CMD, POWER, CAM_TURN_TIME,
                           inverted=inverted, stack_check=False)
                ret = motor_move(ctx, "w", POWER, CAM_FORWARD_TIME,
                                 inverted=inverted, stack_check=True)

            elif order == 3:
                lost_count = 0
                print("ターゲットが左です。左に旋回してから前進します。")
                motor_move(ctx, LEFT_TURN_CMD, POWER, CAM_TURN_TIME,
                           inverted=inverted, stack_check=False)
                ret = motor_move(ctx, "w", POWER, CAM_FORWARD_TIME,
                                 inverted=inverted, stack_check=True)

            # --- スタック / 反転の復帰は motordrive に任せる ---
            handle_stack_result(ctx, ret, inverted=inverted)

            time.sleep(0.1)

        except KeyboardInterrupt:
            raise
        except Exception as e:
            # カメラ系のエラー時は GPS で位置を確認してから継続判断する
            say(f"カメラ等でエラー発生: {e}", "error")
            motor_stop(ctx)

            say("GPSで現在地を確認し、10m圏内かチェックします。")
            near = check_near_goal(ctx)

            if near is True:
                print(f"{NEAR_RADIUS_M:.0f}m圏内を維持しています。近距離フェーズを継続します。")
                time.sleep(0.1)
                continue
            elif near is False:
                if ctx.allow_return_to_phase3:
                    say(f"{NEAR_RADIUS_M:.0f}m圏外に出ました → 遠距離フェーズへ戻ります", "warning")
                    return 3
                say("往復回数の上限に達したため近距離フェーズを継続します", "warning")
                time.sleep(0.1)
                continue
            else:
                say("GPS取得にも失敗しました。近距離フェーズを維持してリトライします。", "warning")
                time.sleep(0.5)
                continue


# ============================================================
# phase5 : ゴールフェーズ
# ============================================================
def run_phase5(ctx):
    say("\n========== phase5 : ゴールフェーズ ==========")
    log_msg("phase", 5)
    motor_stop(ctx)
    say("goal goal goal")
    print("本当におめでとう，そしてありがとう．")
    blink_led(ctx, times=5, interval=0.3)
    return 0   # 0 = 終了


# ============================================================
# 終了処理
# ============================================================
def cleanup(ctx):
    say("\n終了処理中... (Motors, Camera, Sensors)")

    # 1. まずモーターを止める
    if ctx.motor_ok and md is not None:
        try:
            md.stop()
        except Exception:
            pass

    # 2. ニクロム線と LED を確実に OFF (GPIO.cleanup より先に行う)
    if ctx.gpio_ok and GPIO is not None:
        try:
            GPIO.output(NICHROME_PIN, 0)
            GPIO.output(LED_PIN, 0)
        except Exception:
            pass

    # 3. 各デバイスを閉じる
    if ctx.cam is not None:
        try:
            ctx.cam.release()
        except Exception:
            pass

    if ctx.bno is not None:
        try:
            ctx.bno.close()
        except Exception:
            pass

    if ctx.bme is not None:
        try:
            ctx.bme.close()
        except Exception:
            pass

    # 4. motordrive.cleanup() は内部で GPIO.cleanup() を呼ぶ
    if ctx.motor_ok and md is not None:
        try:
            md.cleanup()
        except Exception:
            pass

    # 5. motordrive が無い場合に備えて自前でも GPIO を後始末
    if ctx.gpio_ok and GPIO is not None:
        try:
            GPIO.cleanup()
        except Exception:
            pass

    if cv2 is not None:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

    say("完了。お疲れ様でした。")


# ============================================================
# メイン (状態機械)
# ============================================================
PHASE_FUNCS = {
    1: run_phase1,
    2: run_phase2,
    3: run_phase3,
    4: run_phase4,
    5: run_phase5,
}


def main():
    ctx = setup_devices()
    phase = START_PHASE

    try:
        while phase != 0:
            func = PHASE_FUNCS.get(phase)
            if func is None:
                say(f"未知のフェーズ番号: {phase} → 終了します", "error")
                break

            next_phase = func(ctx)

            # phase4 → phase3 の往復を制限する
            if phase == 4 and next_phase == 3:
                ctx.return_to_phase3_count += 1
                say(f"遠距離フェーズへの復帰 "
                    f"{ctx.return_to_phase3_count}/{MAX_RETURN_TO_PHASE3}", "warning")
                if ctx.return_to_phase3_count >= MAX_RETURN_TO_PHASE3:
                    ctx.allow_return_to_phase3 = False
                    say("以降は近距離フェーズに留まります", "warning")

            if next_phase != 0:
                say(f"\n>>> phase {phase} → phase {next_phase}")
            phase = next_phase

        say("\n========== ミッション終了 ==========")

    except KeyboardInterrupt:
        say("\n中断されました。", "warning")
    except Exception as e:
        say(f"\n予期せぬエラーが発生しました: {e}", "serious_error")
    finally:
        cleanup(ctx)


if __name__ == "__main__":
    main()
