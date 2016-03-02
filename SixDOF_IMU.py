import math
import wpilib
import ITG3200
from ADXL345 import ADXL345
from wpilib._impl.timertask import TimerTask


class SixDOF_IMU:
    def __init__(self, i2c_port,
                 accel_factor=0.07,
                 itg3200_addr=ITG3200.DEFAULT_ADDR,
                 adxl345_addr=None,
                 adxl345_range=ADXL345.Range.k16G):
        self.gyro = ITG3200.ITG3200(i2c_port, addr=itg3200_addr, integrate=False)  # IMU will integrate
        self.accel = ADXL345(i2c_port, accel_range=adxl345_range, address=adxl345_addr)
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0
        self.accel_factor = accel_factor

        self.initGyro()

        self.updateTask = TimerTask("6DOF IMU Update", 50/1000, self.update)

    def initGyro(self):
        self.gyro.init()
        self.gyro.calibrate()

    def free(self):
        self.gyro.free()
        self.accel.free()

    def update(self, dt=50):
        try:
            x = self.accel.getX()
            y = self.accel.getY()
            z = self.accel.getZ()
            # Normalize accelerometer output?
            accel_mag = 1  # (x**2 + y**2 + z**2)**0.5
            self.accel_x = x/accel_mag
            self.accel_y = y/accel_mag
            self.accel_z = z/accel_mag
        except OSError:
            # This happens every once in a while. I don't know why.
            # Maybe fail silently unless it happens a lot?
            wpilib.DriverStation.getInstance().reportError("Error reading from ADXL345", False)

        # Found by looking at my hand
        accel_deg_x = math.degrees(math.atan2(self.accel_z, self.accel_y) + math.pi)
        accel_deg_y = math.degrees(math.atan2(self.accel_z, -self.accel_x) + math.pi)
        accel_deg_z = math.degrees(math.atan2(self.accel_y, self.accel_x) + math.pi)

        gyro_deg_x = self.gyro.getRateX()
        gyro_deg_y = self.gyro.getRateY()
        gyro_deg_z = self.gyro.getRateZ()

        # Complementary filter
        self.angleX = (1-self.accel_factor)*(self.angleX + gyro_deg_x * dt) + self.accel_factor * accel_deg_x
        self.angleY = (1-self.accel_factor)*(self.angleY + gyro_deg_y * dt) + self.accel_factor * accel_deg_y
        self.angleZ = (1-self.accel_factor)*(self.angleZ + gyro_deg_z * dt) + self.accel_factor * accel_deg_z

    def getAngleX(self):
        return self.angleX

    def getAngleY(self):
        return self.angleY

    def getAngleZ(self):
        return self.angleZ

    def resetAngle(self):
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0
