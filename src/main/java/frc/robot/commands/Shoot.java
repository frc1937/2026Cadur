package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.rpsToMps;
import static frc.robot.RobotContainer.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.SHOOTER_LENGTH_METERS;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_CENTER_TURRET;

public class Shoot extends Command {
    private final java.util.List<SimulatedBall> activeBalls = new java.util.ArrayList<>();

    private static final double BALL_RADIUS = 0.075;

    // Flywheel traction coefficients (used to derive initial backspin)
    private static final double TOP_TRACTION_COEFFICIENT    = 0.8;
    private static final double BOTTOM_TRACTION_COEFFICIENT = 1.0;

    /**
     * Height of the turret pivot above the floor (m).
     * Used until ROBOT_TO_CENTER_TURRET is given a real Z value.
     */
    private static final double SHOOTER_HEIGHT = 0.5;

    private int loopCounter = 0;

    public Shoot() {}

    @Override
    public void execute() {
        if (!TURRET.isReadyToShoot()) return;

        // Spawn a ball every 5 loops (~10 balls/s at 50 Hz)
        if (loopCounter % 5 == 0) {
            spawnBall();
        }
        loopCounter++;

        activeBalls.removeIf(ball -> !ball.isActive());
        for (var ball : activeBalls) {
            ball.update();
        }

        Logger.recordOutput("SimulatedBalls",
                activeBalls.stream().map(SimulatedBall::getPose).toArray(Pose3d[]::new));
    }

    private void spawnBall() {
        Pose2d robotPose    = POSE_ESTIMATOR.getPose();
        double robotHeading = robotPose.getRotation().getRadians();

        double phi   = HOOD.getCurrentPosition().getRadians();           // hood elevation angle
        double theta = TURRET.getSelfRelativePosition().getRadians();    // turret yaw in robot frame

        // Turret pointing direction in the field frame
        double fieldTurretAngle = robotHeading + theta;

        double launchSpeed = rpsToMps(FLYWHEEL.getFlywheelVelocity(), Units.inchesToMeters(2.4));

        // --- Ball exit velocity in field frame ---
        double cosHood = Math.cos(phi);
        double sinHood = Math.sin(phi);

        double vx_field = launchSpeed * cosHood * Math.cos(fieldTurretAngle);
        double vy_field = launchSpeed * cosHood * Math.sin(fieldTurretAngle);
        double vz       = launchSpeed * sinHood;

        // Add robot's field-relative velocity so the ball carries the robot's momentum
        ChassisSpeeds fieldRelVel = SWERVE.getFieldRelativeVelocity();
        Translation3d finalVel = new Translation3d(
                vx_field + fieldRelVel.vxMetersPerSecond,
                vy_field + fieldRelVel.vyMetersPerSecond,
                vz
        );

        // --- Spawn position: turret pivot (from ROBOT_TO_CENTER_TURRET) + shooter barrel exit ---
        Pose3d turretPivot = new Pose3d(robotPose).transformBy(ROBOT_TO_CENTER_TURRET);

        double startX = turretPivot.getX() + SHOOTER_LENGTH_METERS * cosHood * Math.cos(fieldTurretAngle);
        double startY = turretPivot.getY() + SHOOTER_LENGTH_METERS * cosHood * Math.sin(fieldTurretAngle);
        // Z: use SHOOTER_HEIGHT for the pivot (ROBOT_TO_CENTER_TURRET has no real Z yet),
        //    then project the barrel upward by the hood elevation.
        double startZ = SHOOTER_HEIGHT + SHOOTER_LENGTH_METERS * sinHood;

        // --- Backspin axis (perpendicular to shot direction, horizontal) ---
        // For a ball launched at fieldTurretAngle, the backspin axis is 90° CCW in the XY plane.
        Translation3d spinAxis = new Translation3d(
                -Math.sin(fieldTurretAngle),
                 Math.cos(fieldTurretAngle),
                 0.0
        );

        // Backspin magnitude derived from differential traction between top/bottom flywheel wheels
        double spinConstant = (BOTTOM_TRACTION_COEFFICIENT - TOP_TRACTION_COEFFICIENT)
                            / (BOTTOM_TRACTION_COEFFICIENT + TOP_TRACTION_COEFFICIENT);
        double initialSpin = (2.0 * spinConstant * launchSpeed) / BALL_RADIUS;

        activeBalls.add(new SimulatedBall(
                new Pose3d(startX, startY, startZ, new Rotation3d()),
                finalVel,
                initialSpin,
                spinAxis
        ));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
