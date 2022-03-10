import wpilib as wp
import ctre
import wpilib.drive
from networktables import NetworkTables
from auto_recorder import AutoRecorder
from wpimath.controller import PIDController
from navx import AHRS

class Robot(wp.TimedRobot):
    def __init__(self):
        self.period = 0.01
        super().__init__(self.period)

    def robotInit(self):

        # Camera Vision
        # wp.CameraServer.launch()
        
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
        self.table = NetworkTables.getTable("limelight")

        # self.stick
        self.stick = wp.Joystick(0)

        # make navx thing
        self.navx = AHRS.create_spi()

        # Auto-recorder
        self.autorecorder = AutoRecorder([self.lfm,  self.lrm, self.rfm, self.rrm])

        # SmartDashboard
        self.sd = NetworkTables.getTable("SmartDashboard")

        # tracking pid controller
        kP = 0.6
        kI = 0
        kD = 0.02
        self.trackingPID = PIDController(kP, kI, kD, self.period)

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
        self.visionTrack()

    def autonomousInit(self):
        # Exception for Update error
        self.robot_drive.setSafetyEnabled(False)
        self.table.putNumber("ledMode", 3)
        
    def teleopPeriodic(self):
        
        if self.stick.getRawButton(1):
            self.table.putNumber("ledMode", 3)
            self.visionTrack()

        else:
            ySpeed = self.square(self.stick.getY()) * self.maxSpeed
            xSpeed = self.square(self.stick.getX() * -1 ) * self.maxSpeed
            zSpeed = self.square(self.stick.getZ() * -1) * self.maxSpeed
            self.robot_drive.driveCartesian(ySpeed, xSpeed, zSpeed) 
            self.autorecorder.recordAuto()
            self.table.putNumber("ledMode", 1)
            

    def visionTrack(self):
        tv = self.table.getNumber('tv', 0)
        if tv == 1:
            tx = self.table.getNumber('tx', 0) / 29.8
            self.smartBoard.putNumber("TEE EX", tx)
            # if abs(tx) > 0.01:
            # self.robot_drive.driveCartesian(0, 0, -self.square(tx))
            zRotationCorrection = self.trackingPID.calculate(tx, 0)
            self.robot_drive.driveCartesian(0, 0, zRotationCorrection)
        else:
            self.robot_drive.driveCartesian(0, 0, 0)

        # ty = self.table.getNumber('ty', 0)
        # ta = self.table.getNumber('ta', 0)
        # ts = self.table.getNumber('ts', 0)
            

if __name__ == '__main__':
    wp.run(Robot)
