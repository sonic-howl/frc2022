import rev
import wpilib as wp
import ctre
import wpilib.drive
from networktables import NetworkTables
from auto_recorder import AutoRecorder
from wpimath.controller import PIDController
from navx import AHRS


def square(x):
    return x * abs(x)


def dz(y, dz=0.05):
    if abs(y) > dz:
        return y
    return 0


def clamp(y, min, max):
    if y > max:
        return max
    if y < min:
        return min
    return y

#  Pickup System for bot
class PickUp():
    def __init__(self):
        self.raising = ctre.WPI_TalonSRX(5)
        self.spining = rev.CANSparkMax(12, rev.CANSparkMax.MotorType.kBrushless)
        # uls = upper lower switch and lls = lower upper switch
        self.uLs = wp.DigitalInput(2)
        self.lLs = wp.DigitalInput(0)
    
    def Raise(self, speed):
        if speed > 0 and self.lLs.get() == False:
            self.raising.set(speed)
        elif speed < 0 and self.uLs.get() == True:
            self.raising.set(speed)
        else:
            self.raising.set(0)
    
    def spin(self, speed):
        self.spining.set(speed)
        
class Robot(wp.TimedRobot):
    def __init__(self):
        self.period = 0.01
        super().__init__(self.period)

    def robotInit(self):

        # Camera Vision
        wp.CameraServer.launch()

        self.smartBoard = NetworkTables.getTable("SmartDashboard")
        self.maxSpeed = 1
        self.smartBoard.putNumber("Max Speed", 1)

        # Initialized the motors
        self.lfm = ctre.WPI_TalonFX(3)
        self.rfm = ctre.WPI_TalonFX(2)
        self.lfm.setInverted(True)
        self.lrm = ctre.WPI_TalonFX(4)
        self.rrm = ctre.WPI_TalonFX(1)
        self.lrm.setInverted(True)
        self.robot_drive = wpilib.drive.MecanumDrive(self.lfm,  self.lrm, self.rfm, self.rrm)
        # this motor is for the shooter
        self.shooter = rev.CANSparkMax(11, rev.CANSparkMax.MotorType.kBrushless)
        # self.hookMotor = ctre.WPI_TalonSRX(8)
        # self.pullMotor = ctre.WPI_TalonSRX(7)

        # limelight table
        self.limelight = NetworkTables.getTable("limelight")
        self.limelight.putNumber("ledMode", 1)

        # self.stick
        self.stick = wp.Joystick(0)
        self.controller = wp.XboxController(1)

        # make navx thing
        self.navx = AHRS.create_spi(update_rate_hz=100)
        # self.gyroPID = PIDController(0.022, 0, 0.0018, self.period)
        self.gyroPID = PIDController(0.04, 0, 0.0035, self.period)
        self.gyroPID.enableContinuousInput(-180, 180)


        # SmartDashboard
        self.sd = NetworkTables.getTable("SmartDashboard")

        # tracking pid controller
        self.trackingYSpeedPID = PIDController(0.2, 0, 0.06, self.period)
        self.trackingRotationPID = PIDController(0.5, 0, 0.035, self.period)
        self.tractionPID = PIDController(0.02, 0, 0.001, self.period)
        self.tractionPID.enableContinuousInput(-180, 180)
        self.lastAngle = 0

        # timer
        self.timer = wp.Timer()
        self.timer.start()

        # pipeline select board
        self.smartBoard.putNumber("pipeline", 0)

        # select recording
        self.smartBoard.putNumber("Autorecord counter", 0)

        self.pickUp = PickUp()
    
        # Auto-recorder
        self.autorecorder = AutoRecorder([self.lfm,  self.lrm, self.rfm, self.rrm, self.pickUp.raising, self.pickUp.spining, self.shooter], self.period)
        self.autorecorder.loadAuto()
        
    def robotPeriodic(self):
        if self.timer.hasPeriodPassed(0.5):
            self.sd.putNumber("Angle", self.navx.getYaw())

        self.maxSpeed = self.smartBoard.getNumber("Max Speed", 1)
        if self.stick.isConnected():
            if self.stick.getRawButton(4):
                self.lastAngle = self.navx.getYaw()
            if self.stick.getRawButton(5):
                self.navx.reset()
        
        self.smartBoard.putBoolean('limitswitch uLs', self.pickUp.uLs.get())
        self.smartBoard.putBoolean('limitswitch lLs', self.pickUp.lLs.get())



    def disabledPeriodic(self):
        self.limelight.putNumber("pipeline", self.smartBoard.getNumber("pipeline", 0))
        self.autorecorder.auto_recording_counter = self.smartBoard.getNumber("Autorecord counter", 0)

    def autonomousInit(self):
        # Exception for Update error
        self.robot_drive.setSafetyEnabled(False)
        # self.table.putNumber("ledMode", 3)

    def autonomousPeriodic(self):
        self.autorecorder.playAuto()
        # self.visionTrack()

    def teleopPeriodic(self):
        # shooting ball
        lTS = -self.controller.getLeftTriggerAxis()
        rTS = self.controller.getRightTriggerAxis()
        speed = -(lTS + rTS)
        self.shooter.set(speed)

        if self.controller.getAButton():
            self.shooter.set(-0.7)

        # if self.controller.getLeftBumper():
        #     self.hookMotor.set(0.25)
        # elif self.controller.getRightBumper():
        #     self.pullMotor.set(0.80)

        zRotationCorrection = 0
        ySpeedCorrection = 0
        # listen for pov
        pov = self.stick.getPOV()
        if pov != -1:
            zRotationCorrection = -self.gyroPID.calculate(self.navx.getYaw(), pov)
            zRotationCorrection = clamp(zRotationCorrection, -0.35, 0.35)
        elif self.stick.getRawButton(1):
            ySpeedCorrection, zRotationCorrection = self.visionTrack()
        else:
            self.limelight.putNumber("ledMode", 1)

        ySpeed = square(dz(self.stick.getY())) * self.maxSpeed + ySpeedCorrection
        xSpeed = square(dz(self.stick.getX()) * -1) * self.maxSpeed
        zSpeed = square(dz(self.stick.getZ()) * -1) * self.maxSpeed + zRotationCorrection
        # ySpeed = square(self.stick.getRawAxis(1)) * self.maxSpeed + ySpeedCorrection
        # xSpeed = square(self.stick.getRawAxis(0) * -1 ) * self.maxSpeed
        # zSpeed = square(self.stick.getRawAxis(4) * -1) * self.maxSpeed + zRotationCorrection

        if zSpeed == 0:
            zSpeed = -self.tractionPID.calculate(self.navx.getYaw(), self.lastAngle)
            zSpeed = dz(zSpeed, 0.07)

        else:
            self.lastAngle = self.navx.getYaw()

        # set starting angle here
        startingAngle = 0
        # UNTESTED
        # this is meant to fix the problem with the vision tracking moving the wrong way because of field oriented controls
        fieldOrientedAngle = self.navx.getAngle() + startingAngle if zRotationCorrection == 0 else 0
        # print(zRotationCorrection, fieldOrientedAngle)
        self.robot_drive.driveCartesian(ySpeed, xSpeed, zSpeed, fieldOrientedAngle)
        self.autorecorder.recordAuto()

        self.pickUp.Raise(self.controller.getLeftY())
        self.pickUp.spin(self.controller.getRightY())


    def visionTrack(self):
        # 1 is for tracking blue, 2 for red
        if self.limelight.getNumber("pipeline", 0) != 1:
            self.limelight.putNumber("ledMode", 3)
        else:
            self.limelight.putNumber("ledMode", 0)

        tv = self.limelight.getNumber('tv', 0)

        if tv == 1:
            tx = self.limelight.getNumber('tx', 0) / 29.8
            ta = self.limelight.getNumber('ta', 100)
            # self.smartBoard.putNumber("TEE EX", tx)
            z = self.trackingRotationPID.calculate(tx, 0)
            y = -self.trackingYSpeedPID.calculate(ta, 3)
            return (y, z)

        return (0, 0)


if __name__ == '__main__':
    wp.run(Robot)
