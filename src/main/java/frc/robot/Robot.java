package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.commands.VisualizeShot;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.HOOD_ANGLE_TO_SHOOTER_LENGTH;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_CENTER_TURRET;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;
import static frc.robot.utilities.PathingConstants.initializeBLine;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        initializeBLine();

        robotContainer = new RobotContainer();
        Threads.setCurrentThreadPriority(true, 99);

        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        HardwareManager.update();
        commandScheduler.run();

        printExactExitDistance();

        POSE_ESTIMATOR.periodic();
        SHOOTING_CALCULATOR.clearLatestParameters();
    }

    private void printExactExitDistance() {
        var hub = HUB_TOP_POSITION.get();

        var robotPose = POSE_ESTIMATOR.getPose();
        var turretPose = new Pose3d(robotPose).transformBy(ROBOT_TO_CENTER_TURRET);

        var turretToHoodExit = new Transform3d(
                new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(HOOD.getCurrentPosition().getRotations()), 0, 0),
                new Rotation3d(0, HOOD.getCurrentPosition().getRadians(), TURRET.getSelfRelativePosition().getRadians())
        );

        var exitPose = turretPose.transformBy(turretToHoodExit);

        Logger.recordOutput("Shooter/ExitPoseDistanceFromHub", hub.getDistance(exitPose.getTranslation()));
        Logger.recordOutput("Shooter/ExitPose", exitPose);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            commandScheduler.schedule(autonomousCommand);

        CommandScheduler.getInstance().schedule(LEDS.showFor(Leds.LEDMode.AUTO_START, 20));
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }

        MATCH_TRACKER.initialize();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();

        robotContainer.updateComponentPoses();
        VisualizeShot.tick();
    }
}