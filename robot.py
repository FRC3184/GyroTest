import wpilib
import ITG3200
import ADXL345
from SixDOF_IMU import SixDOF_IMU


class MyRobot(wpilib.SampleRobot):
    def robotInit(self):
        self.imu = SixDOF_IMU(wpilib.I2C.Port.kOnboard, accel_factor=0.0000)
        wpilib.Timer.delay(50/1000)
        self.imu.calibrate()
        self.imu.resetAngle()

        self.joystick = wpilib.Joystick(1)

    def operatorControl(self):
        maxY = 0
        minY = 0
        while self.isEnabled() and self.isOperatorControl():
            if self.joystick.getRawButton(1):
                self.imu.resetAngle()
            dt = 20/1000

            wpilib.SmartDashboard.putNumber("IMU Angle X", self.imu.getAngleX())
            wpilib.SmartDashboard.putNumber("IMU Angle Y", self.imu.getAngleY())
            wpilib.SmartDashboard.putNumber("IMU Angle Z", self.imu.getAngleZ())

            # wpilib.SmartDashboard.putNumber("IMU Rate X", self.imu.getRateX())
            # wpilib.SmartDashboard.putNumber("IMU Rate Y", self.imu.getRateY())
            # wpilib.SmartDashboard.putNumber("IMU Rate Z", self.imu.getRateZ())

            wpilib.Timer.delay(dt)

if __name__ == '__main__':
    wpilib.run(MyRobot)
