import wpilib as wp
import ctre
import wpilib.drive
from networktables import NetworkTables
from auto_recorder import AutoRecorder
from wpimath.controller import PIDController
from navx import AHRS

class Robot(wpilib.TimedRobot):
    def robotInit(self):

        # Camera Vision
        wp.CameraServer.launch()
        
        self.smartBoard = NetworkTables.getTable("SmartDashboard")
        self.maxSpeed = 1
        self.smartBoard.putNumber("Max Speed" , 1)

        # Initialized the motors
        self.lfm = ctre.WPI_TalonFX(3)
        self.rfm = ctre.WPI_TalonFX(5)
        self.lfm.setInverted(True)
        self.lrm = ctre.WPI_TalonFX(4)
        self.rrm = ctre.WPI_TalonFX(2)
        self.lrm.setInverted(True)
        self.robot_drive = wpilib.drive.MecanumDrive(self.lfm,  self.lrm, self.rfm, self.rrm)
        self.stick = wp.Joystick(0)

        # self.stick

        # make navx thing
        self.navx = AHRS.create_spi()

        # Auto-recorder
        self.autorecorder = AutoRecorder([self.lfm,  self.lrm, self.rfm, self.rrm])

        # SmartDashboard
        self.sd = NetworkTables.getTable("SmartDashboard")

        # timer
        self.timer = wp.Timer()
        self.timer.start()

    def robotPeriodic(self):
        if self.timer.hasPeriodPassed(0.5):
            self.sd.putNumber("Angle", self.navx.getAngle())

        self.maxSpeed = self.smartBoard.getNumber("Max Speed", 1) 
    
    def square(self, x):
        return x * abs(x)

    def autonomousPeriodic(self):
        self.autorecorder.playAuto()

    def autonomousInit(self):
        # Exception for Update error
        self.robot_drive.setSafetyEnabled(False)
        
    def teleopPeriodic(self):
        ySpeed = self.square(self.stick.getY()) * self.maxSpeed
        xSpeed = self.square(self.stick.getX() * -1 ) * self.maxSpeed
        zSpeed = self.square(self.stick.getZ() * -1) * self.maxSpeed

        difference = 0
        # set up traction control using gyro angle and stick angle
        if self.stick.getMagnitude() > 0.1:
            joystickAngle = self.stick.getDirectionDegrees()
            gyroAngle = self.navx.getAngle()
            difference = joystickAngle - gyroAngle
            difference /= 180

        self.sd.putNumber("Joystick Angle", joystickAngle)

        self.robot_drive.driveCartesian(ySpeed, xSpeed, zSpeed + difference, 0)
        self.autorecorder.recordAuto()

if __name__ == '__main__':
    wpilib.run(Robot)