import wpilib.simulation
from pyfrc.physics import core
from pyfrc.physics import drivetrains
from pyfrc.physics.units import units

from robot import Robot

class PhysicsEngine:
    def __init__(self, physics_controller: core.PhysicsInterface, robot_obj: Robot):
        self.physics_controller = physics_controller
        self.drivetrain = drivetrains.MecanumDrivetrain(31 * units.inches, 28 * units.inches, 10 * units.fps)
        self.robot = robot_obj
        # self.gyro = wpilib.simulation.AnalogGyroSim(0)

    def update_sim(self, now, tm_diff):
        lfs = -self.robot.lfm.get()
        rfs = -self.robot.rfm.get()
        lrs = -self.robot.lrm.get()
        rrs = -self.robot.rrm.get()
        # print(lfs, rfs, lrs, rrs)
        speeds = self.drivetrain.calculate(lfs, lrs, -rfs, -rrs)
        pose = self.physics_controller.drive(speeds, tm_diff)

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.robot.navx.setAngleAdjustment(-pose.rotation().degrees() + 5)
        