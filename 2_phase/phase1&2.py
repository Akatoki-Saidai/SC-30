#待機フェーズ＆落下フェーズ
import time
import cv2
import sys
import math
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
try:
    from camera import Camera
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
    from gps import idokeido, calculate_distance_and_angle
except Exception as e:
    idokeido = None
    calculate_distance_and_angle = None
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

# ログ用のヘルパー
def log_msg(msg_type, msg_data):
    try:
        if make_csv is not None:
            make_csv.print(msg_type, msg_data)
    except Exception:
        # ロギングは補助的な処理。失敗しても主処理を止めない。
        pass


# ==========================================
# セットアップ
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


# ==========================================
# メイン処理
# ==========================================
def main():

    bno, cam, bme, qnh, motor_ok, gpio_ok = setup_sensors()

    phase = 1

    try:
        while True:

            # ==========================
            # 待機フェーズ
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
            # 落下フェーズ
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
            # 終了フェーズ（★最小修正：phase=3で無限ループしないようにbreak）
            # ==========================
            elif phase == 3:
                print("phase=3 到達 → ループ終了")
                log_msg('msg', 'phase=3 reached, exiting loop')
                break

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
