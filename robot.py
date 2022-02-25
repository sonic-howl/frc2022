import wpilib as wp
import ctre
import wpilib.drive
from networktables import NetworkTables
from auto_recorder import AutoRecorder

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
        self.stick = wpilib.Joystick(0)

        # Auto-recorder
        self.autorecorder = AutoRecorder([self.lfm,  self.lrm, self.rfm, self.rrm])

    def robotPeriodic(self):

        self.maxSpeed = self.smartBoard.getNumber("Max Speed", 1) 
    
    def square(self, x):
        return x*abs(x)

    def autonomousPeriodic(self):
        self.autorecorder.playAuto()

    def autonomousInit(self):
        # Exception for Update error
        self.robot_drive.setSafetyEnabled(False)
        
    def teleopPeriodic(self):
        ySpeed = self.square(self.stick.getY()) * self.maxSpeed
        xSpeed = self.square(self.stick.getX() * -1 ) * self.maxSpeed
        zSpeed = self.square(self.stick.getZ() * -1) * self.maxSpeed
        self.robot_drive.driveCartesian(ySpeed, xSpeed, zSpeed) 
        self.autorecorder.recordAuto()

if __name__ == '__main__':
    wpilib.run(Robot)
