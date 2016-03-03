import wpilib
from SixDOF_IMU import SixDOF_IMU


class MyRobot(wpilib.SampleRobot):
    def robotInit(self):
        self.imu = SixDOF_IMU(wpilib.I2C.Port.kOnboard)
        self.imu.resetAngle()

        self.joystick = wpilib.Joystick(1)

    def operatorControl(self):
        maxY = 0
        minY = 0
        while self.isEnabled() and self.isOperatorControl():
            if self.joystick.getRawButton(1):
                self.imu.resetAngle()
            dt = 20/1000

            wpilib.SmartDashboard.putNumber("IMU Angle X", self.imu.angleX)
            wpilib.SmartDashboard.putNumber("IMU Angle Y", self.imu.angleY)
            wpilib.SmartDashboard.putNumber("IMU Angle Z", self.imu.angleZ)

            wpilib.Timer.delay(dt)

if __name__ == '__main__':
    wpilib.run(MyRobot)
