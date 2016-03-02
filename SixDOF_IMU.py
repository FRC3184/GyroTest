import ITG3200
import math
import struct
from wpilib import ADXL345_I2C


def sign(x):
    if x < 0:
        return -1
    else:
        return 1


class SixDOF_IMU:
    def __init__(self, i2c_port, accel_factor=0.07, itg3200_addr=ITG3200.DEFAULT_ADDR, adxl345_addr=0x53, adxl345_range=ADXL345_I2C.Range.k16G):
        self.gyro = ITG3200.ITG3200(i2c_port, itg3200_addr)
        self.accel = ADXL345_I2C(i2c_port, adxl345_range, adxl345_addr)
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0
        self.accel_factor = accel_factor

    def initGyro(self):
        self.gyro.init()
        self.gyro.calibrate()

    def update(self, dt):
        try:
            x = self.accel.getX()
            y = self.accel.getY()
            z = self.accel.getZ()
            accel_mag = 1  # (x**2 + y**2 + z**2)**0.5
            self.accel_x = x/accel_mag
            self.accel_y = y/accel_mag
            self.accel_z = z/accel_mag
        except OSError:
            print("Error in accel")

        z = self.gyro.readShortFromRegister(ITG3200.RA_GYRO_ZOUT_H, 2)
        k = z
        sig = -1 if z >> 15 == 0b1 else 1
        z = (z & 0b0111111111111111)*sig
        print(bin(z) + " " + bin(k))

        accel_deg_x = math.degrees(math.atan2(self.accel_z, self.accel_y) + math.pi)
        accel_deg_y = math.degrees(math.atan2(self.accel_z, -self.accel_x) + math.pi)
        accel_deg_z = math.degrees(math.atan2(self.accel_y, self.accel_x) + math.pi)

        gyro_deg_x = self.gyro.getRateX() #* sign(accel_deg_x)
        gyro_deg_y = self.gyro.getRateY() #* sign(accel_deg_y)
        gyro_deg_z = self.gyro.getRateZ() #* sign(accel_deg_z)

        self.angleX = (1-self.accel_factor)*(self.angleX + gyro_deg_x * dt) + self.accel_factor * accel_deg_x
        self.angleY = (1-self.accel_factor)*(self.angleY + gyro_deg_y * dt) + self.accel_factor * accel_deg_y
        self.angleZ = (1-self.accel_factor)*(self.angleZ + gyro_deg_z * dt) + self.accel_factor * accel_deg_z

    def resetAngle(self):
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0