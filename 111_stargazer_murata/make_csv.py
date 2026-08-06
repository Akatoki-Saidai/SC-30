import copy
import inspect
import sys
import time
import traceback
import os
import builtins
import atexit
from datetime import datetime


class CSVLogger:

    def __init__(self):

        self.DEBUG = True

        # CSV列定義
        self.msg_types = [
            'time', 'date', 'file', 'func', 'line',
            'serious_error', 'error', 'warning',
            'msg', 'format_exception',

            'phase',
            'gnss_time',
            'lat', 'lon', 'alt',
            'alt_base_press',
            'goal_lat', 'goal_lon',

            'temp', 'press',

            'camera_area',
            'camera_order',
            'camera_center_x',
            'camera_center_y',
            'camera_frame_size_x',
            'camera_frame_size_y',

            'motor_l',
            'motor_r',

            'goal_relative_x',
            'goal_relative_y',
            'goal_relative_angle_rad',
            'goal_distance',

            'accel_all_x',
            'accel_all_y',
            'accel_all_z',

            'accel_line_x',
            'accel_line_y',
            'accel_line_z',

            'mag_x',
            'mag_y',
            'mag_z',

            'gyro_x',
            'gyro_y',
            'gyro_z',

            'grav_x',
            'grav_y',
            'grav_z',

            'euler_x',
            'euler_y',
            'euler_z',

            'nmea'
        ]


        self.DEFAULT_DICT = {
            x: '' for x in self.msg_types
        }


        # ==========================
        # 保存場所
        # ==========================

        # 現在いる場所
        current_dir = os.getcwd()

        # 下にcsvフォルダ作成
        self.log_dir = os.path.join(
            current_dir,
            "csv"
        )

        os.makedirs(
            self.log_dir,
            exist_ok=True
        )


        # ファイル名
        now = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        self.filename = os.path.join(
            self.log_dir,
            f"log_{now}.csv"
        )


        # ヘッダー作成
        with open(
            self.filename,
            "w",
            encoding="utf-8"
        ) as f:

            f.write(
                ",".join(self.msg_types)
                + "\n"
            )


        # ファイルオープン
        self.log_file = open(
            self.filename,
            "a",
            encoding="utf-8"
        )


        builtins.print(
            "Log file created:",
            self.filename
        )


        # 終了時処理登録
        atexit.register(
            self.close
        )


    # --------------------------
    # CSV書き込み
    # --------------------------

    def write(self, msg_type, msg_data):

        try:

            output_dict = copy.copy(
                self.DEFAULT_DICT
            )


            # 特殊データ処理

            if msg_type in [
                'accel_all',
                'accel_line',
                'mag',
                'gyro',
                'grav',
                'euler'
            ]:

                output_dict[msg_type+'_x'] = str(msg_data[0])
                output_dict[msg_type+'_y'] = str(msg_data[1])
                output_dict[msg_type+'_z'] = str(msg_data[2])


            elif msg_type in [
                'goal_relative',
                'camera_center',
                'camera_frame_size'
            ]:

                output_dict[msg_type+'_x'] = str(msg_data[0])
                output_dict[msg_type+'_y'] = str(msg_data[1])


            elif msg_type == "motor":

                output_dict["motor_l"] = str(msg_data[0])
                output_dict["motor_r"] = str(msg_data[1])


            elif msg_type == "lat_lon":

                output_dict["lat"] = str(msg_data[0])
                output_dict["lon"] = str(msg_data[1])


            elif msg_type in self.msg_types:

                output_dict[msg_type] = str(msg_data)


            else:

                output_dict["msg"] = str(msg_data)



            # 時刻

            output_dict["time"] = str(
                time.monotonic()
            )

            output_dict["date"] = datetime.now().strftime(
                "%Y-%m-%d %H:%M:%S.%f"
            )


            # 呼び出し元

            if self.DEBUG:

                frame = inspect.currentframe().f_back

                output_dict["file"] = (
                    frame.f_code.co_filename
                )

                output_dict["func"] = (
                    frame.f_code.co_name
                )

                output_dict["line"] = (
                    frame.f_lineno
                )


            # CSV化

            values = [
                '"' +
                str(output_dict.get(k,""))
                .replace('"','""')
                .replace('\n',' ')
                +
                '"'

                for k in self.msg_types
            ]


            self.log_file.write(
                ",".join(values)
                + "\n"
            )


        except Exception as e:

            builtins.print(
                "CSV Error:",
                e
            )


    # --------------------------
    # 終了処理
    # --------------------------

    def close(self):

        if self.log_file:

            try:

                self.log_file.flush()

                os.fsync(
                    self.log_file.fileno()
                )

                self.log_file.close()

                builtins.print(
                    "CSV closed"
                )

            except:

                pass