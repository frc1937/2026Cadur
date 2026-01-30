package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.RobotContainer.*;

public class Shoot extends Command {
    private double phi;
    private double theta;
    private Pose2d robotPosition;
    private Pose3d ballPosition;
    private double time;

    @Override
    public void initialize() {
        this.time = 0;
        this.phi = HOOD.getCurrentPosition().getRadians();
        this.theta = TURRET.getCurrentTurretPosition().getRadians();
        this.robotPosition = POSE_ESTIMATOR.getCurrentPose();
        this.ballPosition = new Pose3d(
                robotPosition.getX(), robotPosition.getY(), 1,
                new Rotation3d(0, 0, 0)
        );
    }

    @Override
    public void execute() {
        final Translation3d ballTranslation = ballTrajectory(time);
        this.ballPosition = ballPosition.plus(new Transform3d(
                ballTranslation.getX(),
                ballTranslation.getY(),
                ballTranslation.getZ(),
                new Rotation3d(0, 0, 0)));
        Logger.recordOutput("BALL", ballPosition);
        this.time += 0.02;

    }

    @Override
    public boolean isFinished() {
        return ballPosition.getZ() <= 0 /*|| ballPosition.getZ() > 10 ||
               ballPosition.getX() > 20 || ballPosition.getX() < -10||
               ballPosition.getY() > 10 || ballPosition.getY() < -5||
               ballPosition.getTranslation().toTranslation2d().equals(HUB_POSITION.toTranslation2d())*/;
    }

    public Command stopBall() {
        return Commands.runOnce(() -> ballPosition = new Pose3d(POSE_ESTIMATOR.getCurrentPose().getX(), POSE_ESTIMATOR.getCurrentPose().getY(), -100, new Rotation3d(0, 0, 0)));
    }

    private Translation3d ballTrajectory(double time) {
        final double F = rpsToMps(FLYWHEEL.getFlywheelVelocity(), Units.inchesToMeters(4));

        final double z = F * Math.sin(phi);
        final double x = F * Math.cos(phi) * Math.cos(theta);
        final double y = F * Math.cos(phi) * Math.sin(theta);

        final double acceleration = -20;

        return new Translation3d(x, y, z + acceleration * time * 0.02);
    }
}
