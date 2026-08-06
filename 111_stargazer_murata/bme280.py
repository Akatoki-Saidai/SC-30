import time
from smbus2 import SMBus

def s16(x):
    return x-65536 if x&0x8000 else x

class BME280:
    def __init__(self,addr=0x76,bus=1):
        self.addr=addr
        self.bus=SMBus(bus)
        if self.bus.read_byte_data(addr,0xD0)!=0x60:
            raise Exception("BME280 not found")
        self.t_fine=0
        self.read_calibration()
        self.setup()

    def setup(self):
        self.bus.write_byte_data(self.addr,0xF2,1)
        self.bus.write_byte_data(self.addr,0xF4,(2<<5)|(5<<2)|3)

    def read_calibration(self):
        c=self.bus.read_i2c_block_data(self.addr,0x88,24)
        self.T1=c[1]<<8|c[0]
        self.T2=s16(c[3]<<8|c[2])
        self.T3=s16(c[5]<<8|c[4])

        self.P=[]
        self.P.append(c[7]<<8|c[6])
        for i in range(8):
            self.P.append(s16(c[9+i*2]<<8|c[8+i*2]))

    def read_raw(self):
        d=self.bus.read_i2c_block_data(self.addr,0xF7,8)

        p=d[0]<<12|d[1]<<4|d[2]>>4
        t=d[3]<<12|d[4]<<4|d[5]>>4

        return t,p

    def temperature(self,adc):
        v1=(adc/16384-self.T1/1024)*self.T2
        v2=(adc/131072-self.T1/8192)
        v2=v2*v2*self.T3

        self.t_fine=v1+v2
        return self.t_fine/5120

    def pressure(self,adc):
        v1=self.t_fine/2-64000
        v2=v1*v1*self.P[5]/2048
        v2+=v1*self.P[4]*2
        v2=v2/4+self.P[3]*65536

        v1=((self.P[2]*(v1*v1/8192)/8)+(self.P[1]*v1/2))/262144
        v1=(32768+v1)*self.P[0]/32768

        p=((1048576-adc)-v2/4096)*3125
        p=p*2/v1

        return p/100

    def read(self):
        t,p=self.read_raw()
        temp=self.temperature(t)
        pres=self.pressure(p)
        return temp,pres

    def altitude(self,p,qnh):
        return 44330*(1-(p/qnh)**0.1903)


sensor=BME280()

print("気圧取得中")

data=[]

for i in range(50):
    _,p=sensor.read()
    data.append(p)
    time.sleep(0.02)

qnh=sum(data)/len(data)

t,p=sensor.read()

alt=sensor.altitude(p,qnh)

print(f"温度:{t:.2f}℃")
print(f"気圧:{p:.2f}hPa")
print(f"高度:{alt:.2f}m")

sensor.bus.close()