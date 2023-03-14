import math

import commands2
import wpilib
import wpilib.drive
import romi


class RomiDrivetrain:
    kCountsPerRevolution = 1440.0
    kWheelDiameterInch = 2.75591

    def __init__(self) -> None:
        # The Romi has the left and right motors set to
        # PWM channels 0 and 1 respectively
        self.leftMotor = wpilib.Spark(0)
        self.rightMotor = wpilib.Spark(1)

        # The Romi has onboard encoders that are hardcoded
        # to use DIO pins 4/5 and 6/7 for the left and right
        self.leftEncoder = wpilib.Encoder(4, 5)
        self.rightEncoder = wpilib.Encoder(6, 7)

        # Set up the differential drive controller
        self.drive = wpilib.drive.DifferentialDrive(self.leftMotor, self.rightMotor)

        # Set up the RomiGyro
        # self.gyro = romi.RomiGyro()

        # Set up the BuiltInAccelerometer
        # self.accelerometer = wpilib.BuiltInAccelerometer()

        # Use inches as unit for encoder distances
        self.leftEncoder.setDistancePerPulse(
            1 / self.kCountsPerRevolution
        )
        self.rightEncoder.setDistancePerPulse(
            1 / self.kCountsPerRevolution
        )
        self.resetEncoders()

    def arcadeDrive(self, xaxisSpeed, zaxisRotate, squareInputs=True):
        self.drive.arcadeDrive(xaxisSpeed, zaxisRotate, squareInputs)

    def tankDrive(self, leftSpeed, rightSpeed, squareInputs=True):
        self.drive.tankDrive(leftSpeed, rightSpeed, squareInputs)

    def resetEncoders(self):
        self.leftEncoder.reset()
        self.rightEncoder.reset()

    def getLeftEncoderCount(self):
        return self.leftEncoder.get()

    def getRightEncoderCount(self):
        return self.rightEncoder.get()

    def getLeftEncoderRate(self):
        return self.leftEncoder.getRate()

    def getRightEncoderRate(self):
        return self.rightEncoder.getRate()

    def getLeftDistance(self):
        return self.leftEncoder.getDistance()

    def getRightDistance(self):
        return self.rightEncoder.getDistance()
