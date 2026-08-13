# csvの書き込み

import copy
import inspect
import sys
import time
import traceback
import os
import builtins
from datetime import datetime
import make_csv

def print(msg_type : str, msg_data):
    try:
        special_keys = ['accel_all', 'accel_line', 'mag', 'gyro', 'grav', 'euler', 
                        'goal_relative', 'camera_center', 'camera_frame_size', 'motor', 'lat_lon']
        
        # ガード処理
        if msg_type not in msg_types and msg_type not in special_keys:
             output_dict = copy.copy(DEFAULT_DICT)
             output_dict['msg'] = f"UNKNOWN TYPE [{msg_type}]: {msg_data}"
             builtins.print(f"Warning: Unknown msg_type '{msg_type}' in make_csv.py")
        else:
            output_dict = copy.copy(DEFAULT_DICT)
            
            # 特殊キーの展開
            if msg_type in ['accel_all', 'accel_line', 'mag', 'gyro', 'grav', 'euler']:
                if isinstance(msg_data, (list, tuple)) and len(msg_data) >= 3:
                    output_dict[msg_type + '_x'] = str(msg_data[0])
                    output_dict[msg_type + '_y'] = str(msg_data[1])
                    output_dict[msg_type + '_z'] = str(msg_data[2])
                else:
                    output_dict['error'] = f"Invalid format for {msg_type}: {msg_data}"

            elif msg_type in ['goal_relative', 'camera_center', 'camera_frame_size']:
                if isinstance(msg_data, (list, tuple)) and len(msg_data) >= 2:
                    output_dict[msg_type + '_x'] = str(msg_data[0])
                    output_dict[msg_type + '_y'] = str(msg_data[1])

            elif msg_type == 'motor':
                if isinstance(msg_data, (list, tuple)) and len(msg_data) >= 2:
                    output_dict['motor_l'] = str(msg_data[0])
                    output_dict['motor_r'] = str(msg_data[1])
            
            elif msg_type == 'lat_lon':
                 if isinstance(msg_data, (list, tuple)) and len(msg_data) >= 2:
                    output_dict['lat'] = str(msg_data[0])
                    output_dict['lon'] = str(msg_data[1])
            
            else:
                output_dict[msg_type] = str(msg_data)

        # 共通情報
        output_dict['time'] = str(time.monotonic())
        output_dict['date'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

        # デバッグ情報
        if DEBUG or msg_type in ['error', 'serious_error', 'format_exception']:
            try:
                frame = inspect.currentframe().f_back
                output_dict['file'] = str(frame.f_code.co_filename)
                output_dict['func'] = str(frame.f_code.co_name)
                output_dict['line'] = str(frame.f_lineno)
            except Exception:
                pass
            try:
                if msg_type in ['error', 'serious_error']:
                    e_type, e_obj, e_trace = sys.exc_info()
                    if e_obj is not None:
                        f_exp = traceback.format_exception(e_type, e_obj, e_trace)
                        output_dict['format_exception'] = '"' + str(''.join(f_exp)).replace('"', '""') + '"'
            except Exception:
                pass

        # 【修正箇所】msg_typesの定義順に値を取り出してリスト化する（列ズレ防止の決定版）
        clean_values = [
            '"' + str(output_dict.get(k, '')).replace('"', '""').replace('\n', ' ') + '"'
            for k in msg_types
        ]
        output_msg = ','.join(clean_values)

        log_file.write(output_msg + '\n')
        log_file.flush()
        os.fsync(log_file.fileno())

    except Exception as e:
        builtins.print(f"An error occured in printing to csv: {e}")


# ----------------------------
# BME280Sensor クラスの追加
# ----------------------------
# phase1&2.py が期待する API に合わせた最小限の実装を行います:
# - BME280Sensor(debug=False)
# - calib_ok 属性 (True/False)
# - read_all() -> (temp_c, press_hpa, hum)
# - baseline() -> qnh (hPa) : 現在の気圧を返す
# - altitude(pressure_hpa, qnh=1013.25) -> altitude in meters

try:
    # BME280 の I2C レジスタや補正計算は標準的な手法で実装します
    import struct
    try:
        from smbus2 import SMBus
        _HAS_SMBUS = True
    except Exception:
        _HAS_SMBUS = False

    class BME280Sensor:
        # デフォルトアドレスは 0x76 または 0x77
        def __init__(self, address=None, i2c_bus=1, debug=False):
            self.debug = bool(debug)
            self._bus_num = int(i2c_bus)
            self._addr = address if address is not None else 0x76
            self._bus = None
            self.calib_ok = False
            self._t_fine = 0
            self._dig = {}

            if not _HAS_SMBUS:
                builtins.print("BME280: smbus2 not available; sensor disabled")
                return

            try:
                self._bus = SMBus(self._bus_num)
                # チップID を確認
                chipid = self._read_u8(0xD0)
                if chipid not in (0x60,):
                    # try alternate address
                    if address is None and self._addr == 0x76:
                        self._addr = 0x77
                        chipid = self._read_u8(0xD0)
                if chipid != 0x60:
                    if self.debug: builtins.print(f"BME280: Unexpected chip id: {chipid}")
                    return

                # 読み取り用補正パラメータのロード
                self._load_calibration()

                # 計測モード: normal, oversampling 設定
                # Humidity oversamp
                self._write_u8(0xF2, 0x01)  # humidity oversample x1
                # ctrl_meas: temp and press oversamp x1, mode normal(3)
                self._write_u8(0xF4, 0x27)  # 001 001 11 -> t x1 p x1 mode normal
                # config: standby 1000ms, filter off
                self._write_u8(0xF5, 0xA0)  # 101 0 000 -> t_sb=1000ms, filter off

                self.calib_ok = True
                if self.debug: builtins.print("BME280: initialized successfully")

            except Exception as e:
                if self.debug: builtins.print(f"BME280 init error: {e}")
                self.calib_ok = False

        # --------------------
        # 低レベル I2C ヘルパー
        # --------------------
        def _read_u8(self, reg):
            try:
                return int(self._bus.read_byte_data(self._addr, reg) & 0xFF)
            except Exception:
                return None

        def _read_s16(self, reg):
            try:
                lo = self._read_u8(reg)
                hi = self._read_u8(reg+1)
                if lo is None or hi is None: return None
                val = (hi << 8) + lo
                if val & 0x8000:
                    val = -((~val & 0xFFFF) + 1)
                return val
            except Exception:
                return None

        def _read_u16(self, reg):
            try:
                lo = self._read_u8(reg)
                hi = self._read_u8(reg+1)
                if lo is None or hi is None: return None
                return (hi << 8) + lo
            except Exception:
                return None

        def _write_u8(self, reg, val):
            try:
                self._bus.write_byte_data(self._addr, reg, int(val) & 0xFF)
                return True
            except Exception:
                return False

        # --------------------
        # キャリブレーション読み取り
        # --------------------
        def _load_calibration(self):
            # 温度・圧力補正係数
            self._dig = {}
            # 0x88 - 0xA1
            self._dig['dig_T1'] = self._read_u16(0x88)
            self._dig['dig_T2'] = self._read_s16(0x8A)
            self._dig['dig_T3'] = self._read_s16(0x8C)
            self._dig['dig_P1'] = self._read_u16(0x8E)
            self._dig['dig_P2'] = self._read_s16(0x90)
            self._dig['dig_P3'] = self._read_s16(0x92)
            self._dig['dig_P4'] = self._read_s16(0x94)
            self._dig['dig_P5'] = self._read_s16(0x96)
            self._dig['dig_P6'] = self._read_s16(0x98)
            self._dig['dig_P7'] = self._read_s16(0x9A)
            self._dig['dig_P8'] = self._read_s16(0x9C)
            self._dig['dig_P9'] = self._read_s16(0x9E)
            # humidity
            self._dig['dig_H1'] = self._read_u8(0xA1)
            self._dig['dig_H2'] = self._read_s16(0xE1)
            h3 = self._read_u8(0xE3)
            self._dig['dig_H3'] = h3
            e4 = self._read_u8(0xE4)
            e5 = self._read_u8(0xE5)
            e6 = self._read_u8(0xE6)
            self._dig['dig_H4'] = (e4 << 4) | (e5 & 0xF)
            self._dig['dig_H5'] = (e6 << 4) | (e5 >> 4)
            self._dig['dig_H6'] = self._read_s16(0xE7) & 0xFF

        # --------------------
        # 補正アルゴリズム
        # --------------------
        def _compensate_temperature(self, adc_T):
            # adc_T: 20bit
            try:
                dig = self._dig
                var1 = (adc_T / 16384.0 - dig['dig_T1'] / 1024.0) * dig['dig_T2']
                var2 = ((adc_T / 131072.0 - dig['dig_T1'] / 8192.0) ** 2) * dig['dig_T3']
                self._t_fine = int(var1 + var2)
                T = (var1 + var2) / 5120.0
                return T
            except Exception:
                return None

        def _compensate_pressure(self, adc_P):
            try:
                dig = self._dig
                var1 = self._t_fine / 2.0 - 64000.0
                var2 = var1 * var1 * dig['dig_P6'] / 32768.0
                var2 = var2 + var1 * dig['dig_P5'] * 2.0
                var2 = var2 / 4.0 + dig['dig_P4'] * 65536.0
                var1 = (dig['dig_P3'] * var1 * var1 / 524288.0 + dig['dig_P2'] * var1) / 524288.0
                var1 = (1.0 + var1 / 32768.0) * dig['dig_P1']
                if var1 == 0:
                    return None
                p = 1048576.0 - adc_P
                p = ((p - var2 / 4096.0) * 6250.0) / var1
                var1 = dig['dig_P9'] * p * p / 2147483648.0
                var2 = p * dig['dig_P8'] / 32768.0
                p = p + (var1 + var2 + dig['dig_P7']) / 16.0
                return p / 100.0  # hPa
            except Exception:
                return None

        def _compensate_humidity(self, adc_H):
            try:
                dig = self._dig
                h = self._t_fine - 76800.0
                if h == 0:
                    return None
                h = (adc_H - (dig['dig_H4'] * 64.0 + dig['dig_H5'] / 16384.0 * h)) * (dig['dig_H2'] / 65536.0 * (1.0 + dig['dig_H6'] / 67108864.0 * h * (1.0 + dig['dig_H3'] / 67108864.0 * h)))
                h = h * (1.0 - dig['dig_H1'] * h / 524288.0)
                if h > 100:
                    h = 100
                elif h < 0:
                    h = 0
                return h
            except Exception:
                return None

        # --------------------
        # センサ生値読み取り
        # --------------------
        def _read_raw(self):
            # 0xF7..0xFE: press[3], temp[3], hum[2]
            try:
                data = self._bus.read_i2c_block_data(self._addr, 0xF7, 8)
                # data: [p_msb, p_lsb, p_xlsb, t_msb, t_lsb, t_xlsb, h_msb, h_lsb]
                adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
                adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
                adc_h = (data[6] << 8) | data[7]
                return adc_t, adc_p, adc_h
            except Exception:
                return None, None, None

        # --------------------
        # 公開 API
        # --------------------
        def read_all(self):
            """(temp_C, pressure_hPa, humidity)
            pressure_hPa is in hPa (millibar)
            """
            if not self.calib_ok:
                return None, None, None
            try:
                adc_t, adc_p, adc_h = self._read_raw()
                if adc_t is None:
                    return None, None, None
                temp = self._compensate_temperature(adc_t)
                press = self._compensate_pressure(adc_p)
                hum = self._compensate_humidity(adc_h)
                return temp, press, hum
            except Exception:
                return None, None, None

        def baseline(self, samples=5, delay=0.1):
            """現在の気圧を qnh として返す（hPa）。サンプル平均を返す。"""
            if not self.calib_ok:
                return 1013.25
            vals = []
            for _ in range(max(1, int(samples))):
                _, p, _ = self.read_all()
                if p is not None:
                    vals.append(p)
                time.sleep(delay)
            if not vals:
                return 1013.25
            return sum(vals) / len(vals)

        def altitude(self, pressure_hpa, qnh=1013.25):
            """qnh (hPa) と現在の pressure_hpa から高度(m)を計算する簡易式"""
            try:
                if pressure_hpa is None:
                    return None
                # 国際標準大気に基づく近似式
                return 44330.0 * (1.0 - (float(pressure_hpa) / float(qnh)) ** (1.0 / 5.255))
            except Exception:
                return None

        def close(self):
            try:
                if self._bus is not None:
                    self._bus.close()
            except Exception:
                pass

    # end class

except Exception as e:
    builtins.print(f"Failed to add BME280Sensor: {e}")
