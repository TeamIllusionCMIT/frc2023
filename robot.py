#!/usr/bin/env python3

from wpilib import TimedRobot, MotorControllerGroup, Joystick, run, CameraServer
from rev import CANSparkMax
from wpilib.drive import DifferentialDrive
from dataclasses import dataclass
from time import time_ns, sleep

@dataclass
class DreamState:
    initial_rotations: float
    auto_begin_time: int = None
    auto_complete: bool = False


class POVDirection:
    UP = (0, 45, 270)
    DOWN = (135, 180, 225)


def cap(num: int, threshold: int):
    """returns a number or a aximum value."""
    return num if num <= threshold else threshold


class Luna(TimedRobot):
    __slots__ = (
        "left_front",
        "left_rear",
        "left",
        "right_front",
        "right_rear",
        "right",
        "myRobot",
        "stick",
        "scissor_elevator",
        "scissor_joints",
        "state",
        "right_front_encoder",
        "left_front_encoder",
        "right_rear_encoder",
        "left_rear_encoder",
    )

    def robotInit(self):
        CameraServer.launch()

        """define all the motors. you probably won't have to edit this ever."""

        self.scissor_elevator = CANSparkMax(5, CANSparkMax.MotorType.kBrushed)
        self.scissor_joints = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)

        self.left_front = CANSparkMax(4, CANSparkMax.MotorType.kBrushless)
        self.left_rear = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        self.left = MotorControllerGroup(self.left_front, self.left_rear)

        self.right_front = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)
        self.right_rear = CANSparkMax(3, CANSparkMax.MotorType.kBrushless)

        self.right = MotorControllerGroup(self.right_front, self.right_rear)
        self.right.setInverted(True)

        self.right_front_encoder = self.right_front.getEncoder()
        self.left_front_encoder = self.left_front.getEncoder()
        self.right_rear_encoder = self.right_rear.getEncoder()
        self.left_rear_encoder = self.left_rear.getEncoder()

        self.state = DreamState(
            initial_rotations=abs(self.right_front_encoder.getPosition())
        )
        # object that handles basic drive operations
        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joystick #0
        self.stick = Joystick(0)
        print("ROBO REDEMPTION")

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with arcade steering"""
        if self.stick.getRawButton(2):
            # self.left(-self.left_front_encoder.getVelocity())
            # self.right(-self.right_front_encoder.getVelocity())
            ...
        else:
            print(self.stick.getRawButton(2))
            self.myRobot.arcadeDrive(
                cap((self.stick.getY() * -1) ** 3, 0.5)  # return max velocity of 0/5
                if self.stick.getTrigger()  # but only if the trigger's on
                else (self.stick.getY() * -1) ** 3,  # if it is, give the actual speed
                cap((self.stick.getZ() * -1) ** 3, 0.5)
                if self.stick.getTrigger()
                else (self.stick.getZ() * -1) ** 3,
                False,
            )  # just make the arcade work
        if self.stick.getPOV() in POVDirection.UP:  # if the stick is generally up
            self.scissor_elevator.set(-0.5 if self.stick.getTrigger() else -0.25)
        elif self.stick.getPOV() in POVDirection.DOWN:  # if the stick is generally down
            self.scissor_elevator.set(0.5 if self.stick.getTrigger() else 0.25)
        elif not (
            self.stick.getPOV() in POVDirection.DOWN
            or self.stick.getPOV() in POVDirection.UP
        ):
            # print("neither")
            self.scissor_elevator.stopMotor()
        if self.stick.getRawButton(3):  # if the stick is generally up
            self.scissor_joints.set(0.5)
            ...  # go_up()
        elif self.stick.getRawButton(4):  # if the stick is generally down
            self.scissor_joints.set(-0.5)
            ...  # go_down()
        elif not (self.stick.getRawButton(3) or self.stick.getRawButton(4)):
            self.scissor_joints.stopMotor()

    def autonomousInit(self) -> None:
        # self.myRobot.stopMotor()
        self.state.auto_begin_time = time_ns()

    def autonomousPeriodic(self) -> None:
        """
        1 rotation = ~18.84954 in.
        """
        if self.state.auto_complete:
            self.myRobot.arcadeDrive(0,0)
            return
        run_speed = 0.75
        self.myRobot.arcadeDrive(run_speed, 0)
        if (time_ns() - self.state.auto_begin_time)*1e-9 >= 1: self.state.auto_complete = True

    def autonomousExit(self) -> None:
        self.myRobot.stopMotor()


if __name__ == "__main__":
    run(Luna)