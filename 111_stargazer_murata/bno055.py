import time
import board
import busio
import adafruit_bno055


class BNO055:
    """Adafruit BNO055 9軸センサー用のラッパークラス"""

    def __init__(self, i2c_bus=None, address=0x28):
        """センサーの初期化"""
        if i2c_bus is None:
            i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c_bus, address=address)

    def get_calibration_status(self):
        """キャリブレーション状態を取得 (sys, gyro, accel, mag)"""
        return self.sensor.calibration_status

    def get_magnetic(self):
        """地磁気データを取得 [uT] (X, Y, Z)"""
        return self.sensor.magnetic

    def get_gyro(self):
        """角速度データを取得 [rad/s] (X, Y, Z)"""
        return self.sensor.gyro

    def get_acceleration(self):
        """加速度データを取得 [m/s²] (X, Y, Z)"""
        return self.sensor.acceleration

    def get_linear_acceleration(self):
        """線形加速度データを取得 (重力除く) [m/s²] (X, Y, Z)"""
        return self.sensor.linear_acceleration

    def get_gravity(self):
        """重力加速度データを取得 [m/s²] (X, Y, Z)"""
        return self.sensor.gravity

    def get_euler(self):
        """オイラー角を取得 [°] (Heading, Roll, Pitch)"""
        return self.sensor.euler

    def get_quaternion(self):
        """四元数を取得 (W, X, Y, Z)"""
        return self.sensor.quaternion

    def get_temperature(self):
        """温度を取得 [°C]"""
        return self.sensor.temperature

    def read_all_and_print(self):
        """全センサーデータを読み出してコンソールに出力"""
        # キャリブレーション状態
        sys_cal, gyro_cal, accel_cal, mag_cal = self.get_calibration_status()
        print(f"Gyro_cal={gyro_cal} Accel_cal={accel_cal} Mag_cal={mag_cal}")

        # 地磁気
        mag = self.get_magnetic()
        if mag is not None:
            print(f"Magnetometer : X={mag[0]:8.2f}  Y={mag[1]:8.2f}  Z={mag[2]:8.2f}")

        # ジャイロ
        gyro = self.get_gyro()
        if gyro is not None:
            print(f"Gyroscope    : X={gyro[0]:8.3f}  Y={gyro[1]:8.3f}  Z={gyro[2]:8.3f}")

        # 加速度
        accel = self.get_acceleration()
        if accel is not None:
            print(f"Acceleration : X={accel[0]:8.3f}  Y={accel[1]:8.3f}  Z={accel[2]:8.3f}")

        # 線形加速度
        linear = self.get_linear_acceleration()
        if linear is not None:
            print(f"Linear Accel : X={linear[0]:8.3f}  Y={linear[1]:8.3f}  Z={linear[2]:8.3f}")

        # 重力加速度
        gravity = self.get_gravity()
        if gravity is not None:
            print(f"Gravity      : X={gravity[0]:8.3f}  Y={gravity[1]:8.3f}  Z={gravity[2]:8.3f}")

        # オイラー角
        euler = self.get_euler()
        if euler is not None:
            print(f"Euler        : Heading={euler[0]:7.2f}  Roll={euler[1]:7.2f}  Pitch={euler[2]:7.2f}")

        # 四元数
        quat = self.get_quaternion()
        if quat is not None:
            print(f"Quaternion   : {quat}")

        # 温度
        temp = self.get_temperature()
        if temp is not None:
            print(f"Temperature  : {temp} °C")

        print("-" * 60)


# --- 使用例 ---
if __name__ == "__main__":
    # インスタンスの生成
    bno = BNO055(address=0x28)

    print("Reading BNO055 data...")

    try:
        while True:
            bno.read_all_and_print()
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nProgram stopped.")