import time
from smbus2 import SMBus


def s16(x):
    return x - 65536 if x & 0x8000 else x


def s8(x):
    return x - 256 if x & 0x80 else x


class BME280:

    def __init__(self, addr=0x76, bus=1):

        self.addr = addr
        self.bus = SMBus(bus)

        # チップ確認
        if self.bus.read_byte_data(addr, 0xD0) != 0x60:
            raise Exception("BME280 not found")

        self.t_fine = 0

        self.read_calibration()

        self.setup()


    def setup(self):

        # humidity
        self.bus.write_byte_data(
            self.addr,
            0xF2,
            1
        )

        # temp x2, pressure x16, normal mode
        self.bus.write_byte_data(
            self.addr,
            0xF4,
            (2 << 5) | (5 << 2) | 3
        )

        # filter x16
        self.bus.write_byte_data(
            self.addr,
            0xF5,
            (4 << 2)
        )


    def read_calibration(self):

        c = []

        c += self.bus.read_i2c_block_data(
            self.addr,
            0x88,
            24
        )

        c.append(
            self.bus.read_byte_data(
                self.addr,
                0xA1
            )
        )

        c += self.bus.read_i2c_block_data(
            self.addr,
            0xE1,
            7
        )


        # Temperature

        self.T1 = c[1] << 8 | c[0]

        self.T2 = s16(
            c[3] << 8 | c[2]
        )

        self.T3 = s16(
            c[5] << 8 | c[4]
        )


        # Pressure

        self.P = []

        self.P.append(
            c[7] << 8 | c[6]
        )

        for i in range(8):

            self.P.append(
                s16(
                    c[9+i*2] << 8 |
                    c[8+i*2]
                )
            )


        # Humidity

        self.H1 = c[24]

        self.H2 = s16(
            c[26] << 8 |
            c[25]
        )

        self.H3 = c[27]

        self.H4 = s16(
            (c[28] << 4)
            |
            (c[29] & 0x0F)
        )

        self.H5 = s16(
            (c[30] << 4)
            |
            (c[29] >> 4)
        )

        self.H6 = s8(c[31])


    def read_raw(self):

        d = self.bus.read_i2c_block_data(
            self.addr,
            0xF7,
            8
        )


        p = (
            d[0] << 12 |
            d[1] << 4 |
            d[2] >> 4
        )


        t = (
            d[3] << 12 |
            d[4] << 4 |
            d[5] >> 4
        )


        h = (
            d[6] << 8 |
            d[7]
        )


        return t,p,h


    def temperature(self, adc):

        v1 = (
            adc / 16384
            -
            self.T1 / 1024
        ) * self.T2


        v2 = (
            adc / 131072
            -
            self.T1 / 8192
        )

        v2 *= v2 * self.T3


        self.t_fine = v1 + v2

        return self.t_fine / 5120


    def pressure(self, adc):

        v1 = self.t_fine / 2 - 64000

        v2 = (
            v1*v1*self.P[5]
            /
            2048
        )

        v2 += (
            v1*self.P[4]*2
        )

        v2 = (
            v2/4
            +
            self.P[3]*65536
        )


        v1 = (
            (
                self.P[2]
                *
                (v1*v1/8192)
                /
                8
            )
            +
            (
                self.P[1]
                *
                v1
                /
                2
            )
        ) / 262144


        v1 = (
            32768+v1
        )*self.P[0]/32768


        p = (
            (1048576-adc)
            -
            v2/4096
        )*3125


        p = p*2/v1


        return p/100


    def humidity(self, adc):

        h = (
            self.H2
            *
            (
                adc
                -
                self.H4*64
            )
            /
            65536
        )


        if h < 0:
            h = 0

        if h > 100:
            h = 100

        return h


    def read(self):

        t,p,h = self.read_raw()

        temp = self.temperature(t)

        pres = self.pressure(p)

        hum = self.humidity(h)


        return temp,pres,hum


    def altitude(self,p,qnh):

        return 44330 * (
            1-(p/qnh)**0.1903
        )


if __name__ == "__main__":


    sensor = BME280()


    # 基準気圧取得

    print("Calibrating...")


    ps=[]

    for i in range(50):

        _,p,_ = sensor.read()

        ps.append(p)

        time.sleep(0.02)


    qnh=sum(ps)/len(ps)


    print(
        f"QNH={qnh:.2f} hPa"
    )


    try:

        while True:

            t,p,h=sensor.read()

            alt=sensor.altitude(
                p,
                qnh
            )


            print(
                f"T={t:.2f} C "
                f"P={p:.2f} hPa "
                f"H={h:.2f}% "
                f"ALT={alt:.2f} m"
            )

            time.sleep(1)

    except KeyboardInterrupt:

        sensor.bus.close()