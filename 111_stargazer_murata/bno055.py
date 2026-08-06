import time
import board
import busio
import adafruit_bno055

# I2C初期化
i2c = busio.I2C(board.SCL, board.SDA)

# BNO055のI2Cアドレスを0x28に設定
sensor = adafruit_bno055.BNO055_I2C(i2c, address=0x28)

print("Reading BNO055 data...")


    # キャリブレーション状態
sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status
print(f"Gyro_cal={gyro_cal} Accel_cal={accel_cal} Mag_cal={mag_cal}")

    # 地磁気（μT）
mag = sensor.magnetic
if mag is not None:
        print(f"Magnetometer : X={mag[0]:8.2f}  Y={mag[1]:8.2f}  Z={mag[2]:8.2f}")

    # ジャイロ（rad/s）
gyro = sensor.gyro
if gyro is not None:
        print(f"Gyroscope    : X={gyro[0]:8.3f}  Y={gyro[1]:8.3f}  Z={gyro[2]:8.3f}")

    # 加速度（m/s²）
accel = sensor.acceleration
if accel is not None:
        print(f"Acceleration : X={accel[0]:8.3f}  Y={accel[1]:8.3f}  Z={accel[2]:8.3f}")

    # 線形加速度（m/s²）
linear = sensor.linear_acceleration
if linear is not None:
        print(f"Linear Accel : X={linear[0]:8.3f}  Y={linear[1]:8.3f}  Z={linear[2]:8.3f}")

    # 重力加速度（m/s²）
gravity = sensor.gravity
if gravity is not None:
        print(f"Gravity      : X={gravity[0]:8.3f}  Y={gravity[1]:8.3f}  Z={gravity[2]:8.3f}")

    # オイラー角（°）
euler = sensor.euler
if euler is not None:
        print(f"Euler        : Heading={euler[0]:7.2f}  Roll={euler[1]:7.2f}  Pitch={euler[2]:7.2f}")

    # 四元数
quat = sensor.quaternion
if quat is not None:
        print(f"Quaternion   : {quat}")

    # 温度（℃）
print(f"Temperature  : {sensor.temperature} °C")