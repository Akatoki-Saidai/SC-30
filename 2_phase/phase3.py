# ============================================================
# enkyori.py
# 遠距離フェーズ
# ============================================================

import time
import math
import threading

from bno055 import BNO055

from gps import (
    idokeido,
    calculate_distance_and_angle,
    ERROR_DISTANCE,
)

import motordrive as md


# ============================================================
# 設定
# ============================================================

# ------------------------------------------------------------
# ゴール座標
# 実際のゴール座標を設定する
# ------------------------------------------------------------

GOAL_LAT = None
GOAL_LON = None


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
FORWARD_TIME = 5.0


# ------------------------------------------------------------
# 旋回
# ------------------------------------------------------------

# 1秒間に約90°旋回する想定
OMEGA_DEG_PER_SEC = 90.0

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
#
# 例：
# STACK_ACCEL_THRESHOLD = 0.5
#
# ただし，フローチャートから具体的な値は分からないため
# ここでは勝手に設定しない。
#
STACK_ACCEL_THRESHOLD = None

# ax, ay, az がすべて閾値以下の状態が
# 1秒続いたらスタック
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

# 前回の遠距離フェーズでは
#
#   角度が正 → d
#   角度が負 → a
#
# だったため，その対応を維持する。
#
# gps.pyでは
#
#   正 = 左
#   負 = 右
#
# となっている。
#
# motordrive.pyの実際の動作が逆なら
# この2つを入れ替える。

LEFT_TURN_CMD = "d"
RIGHT_TURN_CMD = "a"


# ============================================================
# セットアップ
# ============================================================

def setup_devices():

    print("BNO055セットアップ開始")

    bno = None

    try:

        bno = BNO055()

        if not bno.begin():

            print("BNO055初期化失敗")

            bno = None

        else:

            print("BNO055初期化成功")

    except Exception as e:

        print(
            f"BNO055 Setup Error: {e}"
        )

        bno = None


    print("モーターセットアップ開始")

    motor_ok = False

    try:

        md.setup_motors()

        motor_ok = True

        print("モーター初期化成功")

    except Exception as e:

        print(
            f"Motor Setup Error: {e}"
        )


    return bno, motor_ok


# ============================================================
# 角度正規化
# ============================================================

def normalize_angle(angle_deg):

    """
    角度を -180 ～ +180° に変換
    """

    return (
        angle_deg + 180.0
    ) % 360.0 - 180.0


# ============================================================
# 初回センサデータ取得
# ============================================================

def get_initial_sensor_data(bno):

    """
    遠距離フェーズ開始直後。

    フローチャート通り

    ・GPS
    ・加速度
    ・重力加速度
    ・角速度
    ・地磁気

    を取得する。
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

        "accel":
            bno.linear_acceleration(),

        "gravity":
            bno.gravity(),

        "gyro":
            bno.gyroscope(),

        "mag":
            bno.magnetometer(),
    }


# ============================================================
# 遠距離走行中のセンサデータ取得
# ============================================================

def get_running_sensor_data(bno):

    """
    初期姿勢確認後の遠距離ループ。

    フローチャート通り

    ・GPS
    ・加速度
    ・角速度
    ・地磁気

    のみ取得。

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

        "accel":
            bno.linear_acceleration(),

        "gyro":
            bno.gyroscope(),

        "mag":
            bno.magnetometer(),
    }


# ============================================================
# 機体姿勢復帰
# ============================================================

def correct_orientation():

    """
    機体が反転していた場合に
    正しい姿勢へ戻すための動作。

    ----------------------------------------------------------
    重要
    ----------------------------------------------------------

    フローチャートには

        「正しい向きになるよう回転」

    とだけ書かれており，

        ・左右モーターをどう回すか
        ・何秒回すか
        ・どの回転方向で機体が起き上がるか

    は示されていない。

    そのため，ここだけは機体構造に合わせて
    実際のモーター動作を設定する必要がある。

    不明な状態で勝手な動作を設定すると
    実機で危険なので，未設定の状態では
    明示的にエラーとする。
    """

    raise NotImplementedError(
        "機体姿勢復帰時のモーター動作が未設定です。"
        "機体を裏返し状態から正常姿勢へ戻す"
        "具体的なモーター動作をcorrect_orientation()に設定してください。"
    )


# ============================================================
# ゴール方向への旋回
# ============================================================

def turn_by_angle(
    bno,
    angle_deg,
    motor_ok=True
):

    """
    GPSから求めた相対角度だけ旋回する。

    gps.pyでは

        angle > 0
            → 左

        angle < 0
            → 右

    となっている。
    """

    if not motor_ok:

        print(
            "モーターが使用できないため旋回できません"
        )

        return


    # --------------------------------------------------------
    # 15°以内なら旋回しない
    # --------------------------------------------------------

    if abs(angle_deg) <= TURN_TOLERANCE_DEG:

        print(
            "方向差15°以内のため旋回不要"
        )

        return


    # ========================================================
    # BNO055が使えない場合
    # ========================================================

    if bno is None:

        if angle_deg > 0:

            cmd = LEFT_TURN_CMD

        else:

            cmd = RIGHT_TURN_CMD


        turn_time = (
            abs(angle_deg)
            / OMEGA_DEG_PER_SEC
        )


        turn_time = max(
            turn_time,
            MIN_TURN_TIME
        )

        turn_time = min(
            turn_time,
            MAX_TURN_TIME
        )


        md.move(
            cmd,
            power=POWER,
            duration=turn_time,
            is_inverted=False,
            enable_stack_check=False
        )

        return


    # ========================================================
    # BNO055フィードバック旋回
    # ========================================================

    euler = bno.euler()


    if euler is None:

        print(
            "BNO055 Euler角取得失敗"
        )

        return


    start_yaw = euler[0]


    target_yaw = (
        start_yaw + angle_deg
    ) % 360.0


    print(
        f"旋回開始："
        f"現在Yaw={start_yaw:.1f}° / "
        f"目標Yaw={target_yaw:.1f}°"
    )


    # ========================================================
    # 最大3回補正
    # ========================================================

    for attempt in range(
        MAX_TURN_ATTEMPTS
    ):

        euler = bno.euler()


        if euler is None:

            print(
                "Yaw取得失敗"
            )

            break


        current_yaw = euler[0]


        diff = normalize_angle(
            target_yaw
            - current_yaw
        )


        print(
            f"現在Yaw="
            f"{current_yaw:.1f}° / "
            f"残り={diff:.1f}°"
        )


        # ----------------------------------------------------
        # 15°以内なら終了
        # ----------------------------------------------------

        if (
            abs(diff)
            <= TURN_TOLERANCE_DEG
        ):

            print(
                "旋回完了"
            )

            return


        # ----------------------------------------------------
        # 左右決定
        # ----------------------------------------------------

        if diff > 0:

            cmd = LEFT_TURN_CMD

        else:

            cmd = RIGHT_TURN_CMD


        # ----------------------------------------------------
        # 旋回時間
        # ----------------------------------------------------

        turn_time = (
            abs(diff)
            / OMEGA_DEG_PER_SEC
        )


        turn_time = max(
            turn_time,
            MIN_TURN_TIME
        )


        turn_time = min(
            turn_time,
            MAX_TURN_TIME
        )


        print(
            f"旋回補正 "
            f"{attempt + 1}/"
            f"{MAX_TURN_ATTEMPTS} "
            f"{turn_time:.2f}s"
        )


        md.move(
            cmd,
            power=POWER,
            duration=turn_time,
            is_inverted=False,
            enable_stack_check=False
        )


        time.sleep(0.3)


# ============================================================
# 前進 + スタック検知
# ============================================================

def forward_with_stack_check(
    bno,
    duration,
    accel_threshold,
    motor_ok=True
):

    """
    フローチャート：

        モーターを稼働させて

        ax
        ay
        az

        が閾値以下の状態が
        1秒継続する？

                YES
                 ↓
             スタック

    BNO055.linear_acceleration() を使用。
    """


    if not motor_ok:

        print(
            "モーターが使用できません"
        )

        return False


    if bno is None:

        raise RuntimeError(
            "スタック判定にはBNO055が必要です"
        )


    if accel_threshold is None:

        raise ValueError(
            "STACK_ACCEL_THRESHOLDが未設定です"
        )


    if accel_threshold <= 0:

        raise ValueError(
            "STACK_ACCEL_THRESHOLDは"
            "正の値にしてください"
        )


    # ========================================================
    # モーター動作用スレッド
    # ========================================================

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


    motor_thread = threading.Thread(
        target=motor_task,
        daemon=True
    )


    # ========================================================
    # モーター開始
    # ========================================================

    motor_thread.start()


    # ========================================================
    # 加速度監視
    # ========================================================

    low_accel_start = None

    stacked = False


    while motor_thread.is_alive():

        accel = (
            bno.linear_acceleration()
        )


        # ----------------------------------------------------
        # センサ読み取り失敗
        # ----------------------------------------------------

        if accel is None:

            low_accel_start = None

            time.sleep(
                STACK_SAMPLE_TIME
            )

            continue


        ax, ay, az = accel


        print(
            f"Linear Accel "
            f"x={ax:.2f}, "
            f"y={ay:.2f}, "
            f"z={az:.2f}"
        )


        # ----------------------------------------------------
        # xyz全部が閾値以下？
        # ----------------------------------------------------

        low_accel = (

            abs(ax)
            <= accel_threshold

            and

            abs(ay)
            <= accel_threshold

            and

            abs(az)
            <= accel_threshold
        )


        # ====================================================
        # YES
        # ====================================================

        if low_accel:

            # 最初に閾値以下になった時刻
            if low_accel_start is None:

                low_accel_start = (
                    time.monotonic()
                )


            # 1秒以上継続
            elif (

                time.monotonic()
                - low_accel_start

                >= STACK_HOLD_TIME
            ):

                stacked = True


        # ====================================================
        # NO
        # ====================================================

        else:

            low_accel_start = None


        time.sleep(
            STACK_SAMPLE_TIME
        )


    # ========================================================
    # モータースレッド終了待ち
    # ========================================================

    motor_thread.join()


    # ========================================================
    # モーター側エラー
    # ========================================================

    if motor_error:

        raise motor_error[0]


    return stacked


# ============================================================
# スタック復帰
# ============================================================

def recover_from_stuck(
    bno,
    motor_ok=True
):

    """
    フローチャート通り

        スタック検知
             ↓
        3秒後退
             ↓
        60°右旋回
             ↓
        2秒前進
             ↓
        センサデータ取得へ戻る
    """


    if not motor_ok:

        return None, None


    print("")
    print("====================")
    print(" スタック復帰開始")
    print("====================")


    # ========================================================
    # ① 3秒後退
    # ========================================================

    print(
        "3秒後退"
    )


    md.move(
        "s",
        power=POWER,
        duration=BACK_TIME,
        is_inverted=False,
        enable_stack_check=False
    )


    # ========================================================
    # ② 60°右旋回
    # ========================================================

    print(
        "60°右旋回"
    )


    # gps.pyでは
    #
    #   正 = 左
    #   負 = 右
    #
    # なので -60°
    turn_by_angle(
        bno=bno,
        angle_deg=-RECOVERY_TURN_DEG,
        motor_ok=motor_ok
    )


    # ========================================================
    # ③ 2秒前進する直前のGPS
    # ========================================================

    start_lat, start_lon = (
        idokeido()
    )


    # ========================================================
    # ④ 2秒前進
    # ========================================================

    print(
        "2秒前進"
    )


    md.move(
        "w",
        power=POWER,
        duration=RECOVERY_FORWARD_TIME,
        is_inverted=False,
        enable_stack_check=False
    )


    time.sleep(0.5)


    print(
        "スタック復帰終了"
    )


    return (
        start_lat,
        start_lon
    )


# ============================================================
# 遠距離フェーズ本体
# ============================================================

def run_long_distance_phase(
    bno,
    goal_lat,
    goal_lon,
    stack_accel_threshold,
    motor_ok=True
):

    print("")
    print("==========================")
    print(" 遠距離フェーズ開始")
    print("==========================")


    # ========================================================
    #
    # 初期姿勢確認ループ
    #
    # GPS
    # 加速度
    # 重力加速度
    # 角速度
    # 地磁気
    #
    # ↓
    #
    # 重力加速度z方向が負？
    #
    # YES → サブキャリア離脱
    #
    # NO
    # ↓
    # 機体反転検知
    # ↓
    # 正しい向きになるよう回転
    # ↓
    # 最初のセンサ取得へ戻る
    #
    # ========================================================

    while True:

        print("")
        print(
            "--- 初期センサ取得 ---"
        )


        data = (
            get_initial_sensor_data(
                bno
            )
        )


        curr_lat = data["lat"]
        curr_lon = data["lon"]

        accel = data["accel"]
        gravity = data["gravity"]
        gyro = data["gyro"]
        mag = data["mag"]


        # ----------------------------------------------------
        # GPS表示
        # ----------------------------------------------------

        if (
            curr_lat is not None
            and
            curr_lon is not None
        ):

            print(
                f"GPS: "
                f"{curr_lat}, "
                f"{curr_lon}"
            )

        else:

            print(
                "GPS取得失敗"
            )


        # ----------------------------------------------------
        # 加速度
        # ----------------------------------------------------

        if accel is not None:

            print(
                f"加速度: "
                f"{accel}"
            )


        # ----------------------------------------------------
        # 重力
        # ----------------------------------------------------

        if gravity is not None:

            print(
                f"重力加速度: "
                f"{gravity}"
            )

        else:

            print(
                "重力加速度取得失敗"
            )


        # ----------------------------------------------------
        # 角速度
        # ----------------------------------------------------

        if gyro is not None:

            print(
                f"角速度: "
                f"{gyro}"
            )


        # ----------------------------------------------------
        # 地磁気
        # ----------------------------------------------------

        if mag is not None:

            print(
                f"地磁気: "
                f"{mag}"
            )


        # ====================================================
        # GPSが取れていなければ再取得
        # ====================================================

        if (
            curr_lat is None
            or
            curr_lon is None
        ):

            time.sleep(1.0)

            continue


        # ====================================================
        # 重力が取れていなければ再取得
        # ====================================================

        if gravity is None:

            time.sleep(0.5)

            continue


        # ====================================================
        # 重力加速度z方向が負向き？
        # ====================================================

        if gravity[2] < 0:

            # YES

            print(
                "重力z < 0"
            )

            print(
                "機体は正常な向きです"
            )

            break


        # ====================================================
        # NO
        #
        # 機体の反転を検知
        # ====================================================

        print(
            "重力z >= 0"
        )

        print(
            "機体の反転を検知"
        )


        # ====================================================
        # 正しい向きになるよう回転
        # ====================================================

        correct_orientation()


        # ====================================================
        # 姿勢復帰後はbreakしない
        #
        # while True先頭へ戻る
        #
        # GPS
        # 加速度
        # 重力
        # 角速度
        # 地磁気
        #
        # を再取得して
        #
        # gravity[2] < 0
        #
        # をもう一度確認する
        # ====================================================

        time.sleep(0.5)


    # ========================================================
    # 初期GPSを進行方向計算用に保存
    # ========================================================

    prev_lat = curr_lat
    prev_lon = curr_lon


    # ========================================================
    # サブキャリア離脱のため5秒前進
    # ========================================================

    print("")
    print(
        "サブキャリア離脱のため5秒前進"
    )


    if motor_ok:

        md.move(
            "w",
            power=POWER,
            duration=INITIAL_FORWARD_TIME,
            is_inverted=False,
            enable_stack_check=False
        )


    # GPS安定待ち
    time.sleep(1.0)


    # ========================================================
    # GPS失敗回数
    # ========================================================

    gps_fail_count = 0


    # ========================================================
    #
    # 遠距離メインループ
    #
    # ここからは重力加速度を使用しない。
    #
    # ========================================================

    while True:

        print("")
        print(
            "--- 遠距離走行ループ ---"
        )


        # ====================================================
        # GPS
        # 加速度
        # 角速度
        # 地磁気
        #
        # 取得
        # ====================================================

        data = (
            get_running_sensor_data(
                bno
            )
        )


        curr_lat = data["lat"]
        curr_lon = data["lon"]

        accel = data["accel"]
        gyro = data["gyro"]
        mag = data["mag"]


        # ====================================================
        # GPSチェック
        # ====================================================

        if (
            curr_lat is None
            or
            curr_lon is None
        ):

            gps_fail_count += 1


            print(
                f"GPS取得失敗 "
                f"{gps_fail_count}/"
                f"{GPS_FAIL_LIMIT}"
            )


            if (
                gps_fail_count
                >= GPS_FAIL_LIMIT
            ):

                print(
                    "GPS取得失敗が連続しました"
                )

                print(
                    "近距離フェーズへ移行"
                )

                return 4


            time.sleep(1.0)

            continue


        gps_fail_count = 0


        print(
            f"GPS: "
            f"{curr_lat}, "
            f"{curr_lon}"
        )


        # ====================================================
        # ゴールとの距離・方位差算出
        # ====================================================

        distance_m, angle_rad = (
            calculate_distance_and_angle(

                curr_lat,
                curr_lon,

                prev_lat,
                prev_lon,

                goal_lat,
                goal_lon
            )
        )


        # ====================================================
        # GPS計算エラー
        # ====================================================

        if (
            distance_m
            == ERROR_DISTANCE
        ):

            print(
                "距離・角度計算失敗"
            )

            time.sleep(0.5)

            continue


        angle_deg = math.degrees(
            angle_rad
        )


        print(
            f"ゴールまでの距離: "
            f"{distance_m:.2f} m"
        )


        print(
            f"ゴール方向との角度差: "
            f"{angle_deg:.1f}°"
        )


        # ====================================================
        # ゴールとの距離が閾値以下？
        # ====================================================

        if (
            distance_m
            <= GOAL_THRESHOLD_M
        ):

            print("")
            print(
                "ゴール10m圏内に到達"
            )

            print(
                "近距離フェーズへ移行"
            )

            return 4


        # ====================================================
        # NO
        #
        # ゴール方向に機体を旋回
        # ====================================================

        print(
            "ゴール方向へ旋回"
        )


        turn_by_angle(
            bno=bno,
            angle_deg=angle_deg,
            motor_ok=motor_ok
        )


        # ====================================================
        #
        # モーターを稼働させて前進
        #
        # 同時に
        #
        # ax
        # ay
        # az
        #
        # が閾値以下の状態が1秒続くか確認
        #
        # ====================================================

        print(
            "5秒前進"
        )


        stacked = (
            forward_with_stack_check(

                bno=bno,

                duration=FORWARD_TIME,

                accel_threshold=(
                    stack_accel_threshold
                ),

                motor_ok=motor_ok
            )
        )


        # ====================================================
        # スタック？
        # ====================================================

        if stacked:

            print("")
            print(
                "スタック検知"
            )


            # =================================================
            # 3秒後退
            # ↓
            # 60°右旋回
            # ↓
            # 2秒前進
            # =================================================

            recovery_lat, recovery_lon = (
                recover_from_stuck(
                    bno=bno,
                    motor_ok=motor_ok
                )
            )


            # =================================================
            # 2秒前進開始地点を
            # 次回の進行方向ベクトルの始点にする
            # =================================================

            if (
                recovery_lat is not None
                and
                recovery_lon is not None
            ):

                prev_lat = (
                    recovery_lat
                )

                prev_lon = (
                    recovery_lon
                )

            else:

                prev_lat = curr_lat
                prev_lon = curr_lon


            # =================================================
            # フローチャート通り
            #
            # GPS・加速度・角速度・地磁気
            # 取得へ戻る
            # =================================================

            continue


        # ====================================================
        # スタックしていない場合
        #
        # 次のGPS方向計算のため
        # 現在地を前回地点として保存
        # ====================================================

        prev_lat = curr_lat
        prev_lon = curr_lon


        time.sleep(0.1)


# ============================================================
# main
# ============================================================

def main():

    # --------------------------------------------------------
    # 設定確認
    # --------------------------------------------------------

    if (
        GOAL_LAT is None
        or
        GOAL_LON is None
    ):

        print(
            "GOAL_LAT / GOAL_LONを設定してください"
        )

        return


    if (
        STACK_ACCEL_THRESHOLD
        is None
    ):

        print(
            "STACK_ACCEL_THRESHOLDを"
            "実験値に設定してください"
        )

        return


    # --------------------------------------------------------
    # セットアップ
    # --------------------------------------------------------

    bno, motor_ok = (
        setup_devices()
    )


    if bno is None:

        print(
            "BNO055が使用できないため"
            "遠距離フェーズを開始できません"
        )

        return


    if not motor_ok:

        print(
            "モーターが使用できないため"
            "遠距離フェーズを開始できません"
        )

        return


    try:

        # ====================================================
        # 遠距離フェーズ
        # ====================================================

        phase = (
            run_long_distance_phase(

                bno=bno,

                goal_lat=GOAL_LAT,

                goal_lon=GOAL_LON,

                stack_accel_threshold=(
                    STACK_ACCEL_THRESHOLD
                ),

                motor_ok=motor_ok
            )
        )


        # ====================================================
        # phase 4
        # ====================================================

        if phase == 4:

            print("")
            print(
                "=========================="
            )

            print(
                " 近距離フェーズへ移行"
            )

            print(
                "=========================="
            )


    except KeyboardInterrupt:

        print(
            "\nプログラムを中断しました"
        )


    except NotImplementedError as e:

        print("")
        print(
            "姿勢復帰処理が未設定です"
        )

        print(e)


    except Exception as e:

        print(
            f"\nエラー: {e}"
        )


    finally:

        print("")
        print(
            "終了処理"
        )


        # BNO055
        if bno is not None:

            try:

                bno.close()

            except Exception:

                pass


        # Motor
        if motor_ok:

            try:

                md.cleanup()

            except Exception:

                pass


        print(
            "終了しました"
        )


# ============================================================
# 実行
# ============================================================

if __name__ == "__main__":

    main()
