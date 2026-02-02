package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.RobotContainer.*;

public class Shoot extends Command {
    private final java.util.List<SimulatedBall> activeBalls = new java.util.ArrayList<>();

    // Use constants from FuelShootingVisualizationConstants
    private final double BALL_RADIUS = 0.075;

    // Traction coefficients for spin initialization
    private final double TOP_TRACTION_COEFFICIENT = 0.8;
    private final double BOTTOM_TRACTION_COEFFICIENT = 1;

    // Shooter constants
    private final double SHOOTER_HEIGHT = 0.5; // meters, height of shooter off ground
    private final double SHOOTER_OFFSET_X = 0.0; // meters forward from robot center
    private final double SHOOTER_OFFSET_Y = 0.0; // meters left from robot center

    public Shoot() {
        // This allows the command to run continuously while you hold the button
    }

    private int loopCounter = 0;

    @Override
    public void execute() {
        // 1. Machine Gun Limiter: Spawn a ball every 5 loops (approx 10 balls per second)
        if (loopCounter % 5 == 0) {
            spawnBall();
        }
        loopCounter++;

        // 2. Update all existing balls with the new physics
        activeBalls.removeIf(ball -> !ball.isActive());
        for (var ball : activeBalls) {
            ball.update();
        }

        // 3. Log as an array for AdvantageScope 3D view
        Logger.recordOutput("SimulatedBalls", activeBalls.stream().map(SimulatedBall::getPose).toArray(Pose3d[]::new));
    }

    private void spawnBall() {
        // Get current robot and shooter states
        double phi = HOOD.getCurrentPosition().getRadians(); // Vertical angle
        double theta = TURRET.getCurrentPosition().getRadians(); // Horizontal turret angle
        Pose2d robotPose = POSE_ESTIMATOR.getPose();
        double robotHeading = robotPose.getRotation().getRadians();

        // Calculate launch speed
        double launchSpeed = rpsToMps(FLYWHEEL.getFlywheelVelocity(), Units.inchesToMeters(2.4));

        // Calculate velocity components in robot frame
        double vx_shooter = launchSpeed * Math.cos(phi); // Horizontal component
        double vz_shooter = launchSpeed * Math.sin(phi); // Vertical component

        // Rotate horizontal component by turret angle to get robot-relative velocity
        double vx_robot = vx_shooter * Math.cos(theta);
        double vy_robot = vx_shooter * Math.sin(theta);

        // Get robot velocity (already field-relative from your code)
        ChassisSpeeds robotRelVel = SWERVE.getRobotRelativeVelocity();
        ChassisSpeeds fieldRelVel = ChassisSpeeds.fromFieldRelativeSpeeds(
                robotRelVel.vxMetersPerSecond,
                robotRelVel.vyMetersPerSecond,
                robotRelVel.omegaRadiansPerSecond,
                robotPose.getRotation()
        );

        // Rotate robot-relative ball velocity to field coordinates
        double totalHeading = robotHeading; // + theta already accounted in rotation
        double vx_field = vx_robot * Math.cos(totalHeading) - vy_robot * Math.sin(totalHeading);
        double vy_field = vx_robot * Math.sin(totalHeading) + vy_robot * Math.cos(totalHeading);

        // Combine with robot velocity
        Translation3d finalVel = new Translation3d(
                vx_field + fieldRelVel.vxMetersPerSecond,
                vy_field + fieldRelVel.vyMetersPerSecond,
                vz_shooter // No robot vertical velocity to add
        );

        // Calculate shooter position relative to robot center
        double shooterOffsetX = SHOOTER_OFFSET_X * Math.cos(theta) - SHOOTER_OFFSET_Y * Math.sin(theta);
        double shooterOffsetY = SHOOTER_OFFSET_X * Math.sin(theta) + SHOOTER_OFFSET_Y * Math.cos(theta);

        // Rotate offsets to field coordinates
        double fieldOffsetX = shooterOffsetX * Math.cos(robotHeading) - shooterOffsetY * Math.sin(robotHeading);
        double fieldOffsetY = shooterOffsetX * Math.sin(robotHeading) + shooterOffsetY * Math.cos(robotHeading);

        // Calculate starting position
        double startX = robotPose.getX() + fieldOffsetX;
        double startY = robotPose.getY() + fieldOffsetY;
        double startZ = SHOOTER_HEIGHT; // Height off ground

        // Initialize spin based on traction coefficients (like in VisualizeFuelShootingCommand)
        double spinConstant = (BOTTOM_TRACTION_COEFFICIENT - TOP_TRACTION_COEFFICIENT) /
                (BOTTOM_TRACTION_COEFFICIENT + TOP_TRACTION_COEFFICIENT);
        double initialSpin = (2 * spinConstant * launchSpeed) / BALL_RADIUS;

        activeBalls.add(new SimulatedBall(
                new Pose3d(startX, startY, startZ, new Rotation3d()),
                finalVel,
                initialSpin
        ));
    }

    @Override
    public boolean isFinished() {
        return false; // Keep running to manage the balls
    }
}