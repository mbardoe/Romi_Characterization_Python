import csv
import datetime
import os
from wpilib import TimedRobot, Timer, Joystick, RobotController
from Romi_Drivetrain import RomiDrivetrain
import wpilib

HOME_DIRECTORY = "/Users/mbardoe/Downloads/"


class SpeedCalculator:
    def __init__(self, counts_to_full_speed=400, counts_at_full_speed=50, direction=True):
        self.counts_to_full_speed = counts_to_full_speed
        self.counts_at_full_speed = counts_at_full_speed
        if direction:
            self.direction = 1
        else:
            self.direction = -1

    def speed(self, count):
        if count < self.counts_to_full_speed:
            return self.direction * count / self.counts_to_full_speed
        elif count < self.counts_to_full_speed + self.counts_at_full_speed:
            return self.direction * 1
        else:
            return 1 - (count - self.counts_at_full_speed - self.counts_to_full_speed) / self.counts_to_full_speed

    def is_finished(self, count):
        return count > 2 * self.counts_to_full_speed + self.counts_at_full_speed


class Robot(TimedRobot):

    def __init__(self):
        super().__init__()
        self.samples = 40
        self.counter = 0
        self.start_time = 0
        self.prior_auto_speed = 0
        self.speed_calculator = SpeedCalculator()
        self.datacollection=True

        self.drivetrain = RomiDrivetrain()
        self.data_headers = ["Time", "Voltage", "Speed Requext", "Left Motor Volts", "Right Motor Volts",
                             "Left Encoder Pos", "Right Encoder Pos", "Left Rate", "Right Rate",
                             "Left Acc", "Right Acc"]
        self.data = []
        self.auto_speed = 0
        self.data_collection = True

    def robotInit(self) -> None:
        self.stick = Joystick(0)

        self.now = Timer.getFPGATimestamp()

        self.left_encoder_pos = self.drivetrain.getLeftDistance()
        self.left_encoder_rate = self.drivetrain.getLeftEncoderRate()
        self.left_encoder_prev_rate = self.drivetrain.getLeftEncoderRate()
        self.left_encoder_acc = 0

        self.right_encoder_pos = self.drivetrain.getRightDistance()
        self.right_encoder_rate = self.drivetrain.getRightEncoderRate()
        self.right_encoder_prev_rate = self.drivetrain.getRightEncoderRate()
        self.right_encoder_acc = 0

    def robotPeriodic(self) -> None:
        pass
        # print(f"Left Encoder Position: {self.drivetrain.getLeftDistance()}")
        # print(f"Left Encoder Rate: {self.drivetrain.getLeftEncoderRate()}")
        # print(f"Right Encoder Position: {self.drivetrain.getRightDistance()}")
        # print(f"Right Encoder Position: {self.drivetrain.getRightEncoderRate()}")

    def autonomousInit(self) -> None:
        print("Robot in Auto")
        self.drivetrain.resetEncoders()
        self.start_time = Timer.getFPGATimestamp()
        self.prev_time = Timer.getFPGATimestamp()
        self.counter = 0

    def autonomousPeriodic(self) -> None:
        if self.data_collection:
            self.now = Timer.getFPGATimestamp()
            self.left_encoder_pos = self.drivetrain.getLeftDistance()
            self.left_encoder_rate = self.drivetrain.getLeftEncoderRate()
            self.left_encoder_acc = (self.left_encoder_rate - self.left_encoder_prev_rate) / (self.now - self.prev_time)
            self.right_encoder_pos = self.drivetrain.getRightDistance()
            self.right_encoder_rate = self.drivetrain.getRightEncoderRate()
            self.right_encoder_acc = (self.right_encoder_rate - self.right_encoder_prev_rate) / (
                    self.now - self.prev_time)
            self.battery = RobotController.getBatteryVoltage()
            self.motor_volts = self.battery * abs(self.prior_auto_speed)
            self.left_motor_volts = self.motor_volts
            self.right_motor_volts = self.motor_volts
            self.prior_auto_speed = self.auto_speed
            self.counter += 1
            self.auto_speed = self.speed_calculator.speed(self.counter)
            self.prev_time = self.now
            self.left_encoder_prev_rate = self.left_encoder_rate
            self.right_encoder_prev_rate = self.right_encoder_rate

            self.drivetrain.tankDrive(self.auto_speed, -self.auto_speed)

            self.data.append(
                [f'{self.now:4}',
                 f'{self.battery:4}',
                 f'{self.auto_speed:4}',
                 f'{self.left_motor_volts:4}',
                 f'{self.right_motor_volts:4}',
                 f'{self.left_encoder_pos:4}',
                 f'{self.right_encoder_pos:4}',
                 f'{self.left_encoder_rate:4}',
                 f'{self.right_encoder_rate:4}',
                 f'{self.left_encoder_acc:4}',
                 f'{self.right_encoder_acc:4}']
            )

            if self.speed_calculator.is_finished(self.counter):
                self.datacollection = False


    def disabledInit(self) -> None:
        elapsed_time = Timer.getFPGATimestamp() - self.start_time
        print("Robot Disabled")
        self.drivetrain.tankDrive(0, 0)
        # print(os.getcwd())
        print(f"Collected " + str(len(self.data)) + " readings in " + str(elapsed_time) + " seconds")
        filename = HOME_DIRECTORY + 'characdata' + datetime.datetime.now().strftime("%m%d%H%M%S") + '.csv'
        if len(self.data) > 0:
            with open(filename, 'w', newline='') as csvfile:
                datawriter = csv.writer(csvfile, delimiter=',',
                                        quoting=csv.QUOTE_MINIMAL)
                datawriter.writerow(self.data_headers)
                for row in self.data:
                    datawriter.writerow(row)
            csvfile.close()
        print("Completed Data Transfer.")
        self.data = []

    def teleopPeriodic(self) -> None:
        self.drivetrain.arcadeDrive(self.stick.getRawAxis(0), self.stick.getRawAxis(1))
        # print(f"Left Encoder Position: {self.drivetrain.getLeftDistance()}")
        # print(f"Left Encoder Rate: {self.drivetrain.getLeftEncoderRate()}")
        # print(f"Right Encoder Position: {self.drivetrain.getRightDistance()}")
        # print(f"Right Encoder Position: {self.drivetrain.getRightEncoderRate()}")

    def disabledPeriodic(self) -> None:
        self.drivetrain.tankDrive(0, 0)


if __name__ == "__main__":
    os.environ["HALSIMWS_HOST"] = "10.0.0.2"
    os.environ["HALSIMWS_PORT"] = "3300"
    wpilib.run(Robot)
