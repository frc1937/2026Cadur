package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.interpolation.InverseInterpolator.forDouble;
import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.ShootingConstants.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.SHOOTER_LENGTH_METERS;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_CENTER_TURRET;
import static frc.robot.subsystems.shooter.turret.TurretConstants.TURRET_ANGLE_TOLERANCE_ROTATIONS;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class ShootingCalculator {
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));
    private Rotation2d lastTurretAngle;

    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;

    //TODO: Tune these tables lmao
    static {
        DISTANCE_TO_FLYWHEEL_RPS.put(1.34, 33.42);
        DISTANCE_TO_FLYWHEEL_RPS.put(1.78, 35.01);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.17, 35.01);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.81, 36.62);
        DISTANCE_TO_FLYWHEEL_RPS.put(3.82, 39.79);
        DISTANCE_TO_FLYWHEEL_RPS.put(4.09, 40.61);
        DISTANCE_TO_FLYWHEEL_RPS.put(4.40, 41.38);
        DISTANCE_TO_FLYWHEEL_RPS.put(4.77, 42.20);
        DISTANCE_TO_FLYWHEEL_RPS.put(5.57, 43.77);
        DISTANCE_TO_FLYWHEEL_RPS.put(5.60, 46.17);

        DISTANCE_TO_HOOD_ANGLE.put(1.34, Rotation2d.fromDegrees(71.0));
        DISTANCE_TO_HOOD_ANGLE.put(1.78, Rotation2d.fromDegrees(71.0));
        DISTANCE_TO_HOOD_ANGLE.put(2.17, Rotation2d.fromDegrees(66.0));
        DISTANCE_TO_HOOD_ANGLE.put(2.81, Rotation2d.fromDegrees(63.0));
        DISTANCE_TO_HOOD_ANGLE.put(3.82, Rotation2d.fromDegrees(61.0));
        DISTANCE_TO_HOOD_ANGLE.put(4.09, Rotation2d.fromDegrees(60.0));
        DISTANCE_TO_HOOD_ANGLE.put(4.40, Rotation2d.fromDegrees(59.0));
        DISTANCE_TO_HOOD_ANGLE.put(4.77, Rotation2d.fromDegrees(58.0));
        DISTANCE_TO_HOOD_ANGLE.put(5.57, Rotation2d.fromDegrees(58.0));
        DISTANCE_TO_HOOD_ANGLE.put(5.60, Rotation2d.fromDegrees(55.0));

        DISTANCE_TO_TIME_OF_FLIGHT.put(1.34, 0.88);
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.38, 0.90);
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.88, 1.09);
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.15, 1.11);
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.55, 1.12);
        DISTANCE_TO_TIME_OF_FLIGHT.put(5.68, 1.16);
    }

    public record ShootingParameters(boolean isValid, Rotation2d turretAngle, double turretVelocityRotPS,
                                     Rotation2d hoodAngle,
                                     double flywheelRPS) {
    }

    public ShootingParameters getResults() {
        if (latestParameters != null) return latestParameters;

        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY);

        final var target = HUB_TOP_POSITION.get();
        final var turretPosition = new Pose3d(correctedPose).transformBy(ROBOT_TO_CENTER_TURRET);

        var hoodExitPosition = turretPosition;

        // Calculate turret velocity
        ChassisSpeeds robotSpeeds = SWERVE.getFieldRelativeVelocity();
        Rotation2d robotHeading = correctedPose.getRotation();

        double turretRelativeX = ROBOT_TO_CENTER_TURRET.getX();
        double turretRelativeY = ROBOT_TO_CENTER_TURRET.getY();

        double turretFieldX = turretRelativeX * robotHeading.getCos() - turretRelativeY * robotHeading.getSin();
        double turretFieldY = turretRelativeX * robotHeading.getSin() + turretRelativeY * robotHeading.getCos();

        double tangentialVelocityX = -(robotSpeeds.omegaRadiansPerSecond) * turretFieldY;
        double tangentialVelocityY = (robotSpeeds.omegaRadiansPerSecond) * turretFieldX;

        double turretVelocityX = robotSpeeds.vxMetersPerSecond + tangentialVelocityX;
        double turretVelocityY = robotSpeeds.vyMetersPerSecond + tangentialVelocityY;
        //-------------------

        double timeOfFlight = 0;
        double predictedDistance = target.getDistance(turretPosition.getTranslation());

        Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
        Rotation2d turretAngle = target.minus(turretPosition.getTranslation()).toTranslation2d().getAngle();

        Pose3d predictedExitPose = hoodExitPosition;

        Transform3d turretToHoodExit;
        int i;

        for (i = 0; i < MAX_ITERATIONS; i++) {
            turretToHoodExit = new Transform3d(
                    new Translation3d(SHOOTER_LENGTH_METERS, 0, 0),
                    new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians())
            );

            hoodExitPosition = turretPosition.transformBy(turretToHoodExit);

            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);

            final double offsetX = turretVelocityX * timeOfFlight;
            final double offsetY = turretVelocityY * timeOfFlight;

            predictedExitPose = new Pose3d(
                    hoodExitPosition.getTranslation().plus(new Translation3d(offsetX, offsetY, 0)),
                    hoodExitPosition.getRotation());

            final double newDistance = target.getDistance(predictedExitPose.getTranslation());
            final Rotation2d newHoodAngle = DISTANCE_TO_HOOD_ANGLE.get(newDistance);
            final Rotation2d newTurretAngle = target.minus(predictedExitPose.getTranslation()).toTranslation2d().getAngle();

            final boolean converged = Math.abs(newDistance - predictedDistance) < DISTANCE_TOLERANCE_METERS &&
                    Math.abs(newHoodAngle.minus(hoodAngle).getDegrees()) < HOOD_ANGLE_TOLERANCE_DEGREES &&
                    Math.abs(newTurretAngle.minus(turretAngle).getRotations()) < TURRET_ANGLE_TOLERANCE_ROTATIONS;

            predictedDistance = newDistance;
            hoodAngle = newHoodAngle;
            turretAngle = newTurretAngle;

            if (converged) break;
        }

        if (lastTurretAngle == null) lastTurretAngle = turretAngle;

        double targetTurretVelocity = turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRotations() / PERIODIC_TIME_SEC);
        lastTurretAngle = turretAngle;

        boolean inRange = predictedDistance >= MIN_DISTANCE && predictedDistance <= MAX_DISTANCE;

        latestParameters = new ShootingParameters(
                inRange,
                turretAngle,
                targetTurretVelocity,
                hoodAngle,
                DISTANCE_TO_FLYWHEEL_RPS.get(predictedDistance)
        );

        Logger.recordOutput("ShotCalculator/PredictedExitPose", predictedExitPose);
        Logger.recordOutput("ShotCalculator/TargetHoodAngle", hoodAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/TargetTurretAngle", turretAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", predictedDistance);
        Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("ShotCalculator/TurretVelocityX", turretVelocityX);
        Logger.recordOutput("ShotCalculator/TurretVelocityY", turretVelocityY);
        Logger.recordOutput("ShotCalculator/IterationConverged", i);

        return latestParameters;
    }

    public void clearLatestParameters() {
        latestParameters = null;
    }
}
