import serial
import pynmea2


class GPS:

    def __init__(
        self,
        port="/dev/serial0",
        baudrate=115200
    ):

        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=1
        )

        self.data = {
            "lat": None,
            "lon": None,
            "speed": None,
            "course": None,
            "altitude": None,
            "satellites": None,
            "hdop": None,
            "fix": 0
        }

    def update(self):

        while True:

            line = self.ser.readline().decode(
                "ascii",
                errors="ignore"
            ).strip()

            if not line:
                continue

            try:

                # ----------------
                # RMC
                # ----------------
                if line.startswith("$GNRMC"):

                    msg = pynmea2.parse(line)

                    if msg.status == "A":

                        self.data["lat"] = msg.latitude
                        self.data["lon"] = msg.longitude

                        try:
                            self.data["speed"] = float(
                                msg.spd_over_grnd
                            )
                        except:
                            pass

                        try:
                            self.data["course"] = float(
                                msg.true_course
                            )
                        except:
                            pass

                # ----------------
                # GGA
                # ----------------
                elif line.startswith("$GNGGA"):

                    msg = pynmea2.parse(line)

                    try:
                        self.data["fix"] = int(
                            msg.gps_qual
                        )

                        self.data["satellites"] = int(
                            msg.num_sats
                        )

                        self.data["hdop"] = float(
                            msg.horizontal_dil
                        )

                        self.data["altitude"] = float(
                            msg.altitude
                        )

                    except:
                        pass

                # データ取得完了
                if (
                    self.data["lat"] is not None
                    and self.data["lon"] is not None
                    and self.data["fix"] > 0
                ):
                    return self.data

            except pynmea2.ParseError:
                continue

    def get_data(self):
        return self.data

    def close(self):
        self.ser.close()


# --------------------------
# 使用例
# --------------------------
if __name__ == "__main__":

    gps = GPS()

    gps.update()

    data = gps.get_data()

    gps.close()