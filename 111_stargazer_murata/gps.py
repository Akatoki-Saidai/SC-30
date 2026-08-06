import serial
import pynmea2
import time


PORT = "/dev/serial0"
BAUD = 115200


ser = serial.Serial(
    PORT,
    BAUD,
    timeout=1
)


print("GPS single measurement start")


data = {
    "lat": None,
    "lon": None,
    "speed": None,
    "course": None,
    "altitude": None,
    "satellites": None,
    "hdop": None,
    "fix": 0
}


while True:

    line = ser.readline().decode(
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

                data["lat"] = msg.latitude
                data["lon"] = msg.longitude

                try:
                    data["speed"] = float(
                        msg.spd_over_grnd
                    )

                except:
                    pass


                try:
                    data["course"] = float(
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
                data["fix"] = int(
                    msg.gps_qual
                )

                data["satellites"] = int(
                    msg.num_sats
                )

                data["hdop"] = float(
                    msg.horizontal_dil
                )

                data["altitude"] = float(
                    msg.altitude
                )

            except:
                pass



        # RMC + GGA取得完了

        if (
            data["lat"] is not None
            and data["lon"] is not None
            and data["fix"] > 0
        ):

            print("\nGPS DATA")
            print("----------------")
            print(f"Latitude : {data['lat']}")
            print(f"Longitude: {data['lon']}")
            print(f"Speed    : {data['speed']} knot")
            print(f"Course   : {data['course']} deg")
            print(f"Altitude : {data['altitude']} m")
            print(f"Satellites: {data['satellites']}")
            print(f"HDOP     : {data['hdop']}")
            print(f"FIX      : {data['fix']}")

            break


    except pynmea2.ParseError:
        continue


ser.close()

print("\nGPS measurement finished")