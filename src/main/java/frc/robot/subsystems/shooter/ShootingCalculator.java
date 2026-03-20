package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.geometry.Rotation2d.kZero;
import static edu.wpi.first.math.interpolation.InverseInterpolator.forDouble;
import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.ShootingConstants.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.HOOD_ANGLE_TO_SHOOTER_LENGTH;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_CENTER_TURRET;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_CENTER_TURRET_2d;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;
import static java.lang.Math.abs;

public class ShootingCalculator {
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));
    private Rotation2d lastTurretAngle;

    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;

    static {
        final double[][] LUT_ROWS = {
                {1.676013, 11, 40, 1.15, 2.04},
                {1.813362, 18, 40, 0.78, 1.66},
                {2.027652, 25, 40, 3.72, 4.50},
                {2.207077, 28.5, 40, 1.55, 2.32},
                {2.398858, 30, 41, 1.58, 2.35},
                {2.588988, 30, 42.5, 1.35, 2.19},
                {2.805966, 30, 44, 0.75, 1.69},
                {3.002647, 30, 46, 1.14, 2.07},
                {3.206445, 29, 47.3, 1.42, 2.48},
                {3.392430, 29, 48, 1.92, 2.67}, //TODO: Bad ToF. remeasure
                {3.598324, 31, 49, 2.02, 2.8}, //TODO: Bad ToF. remeasure.
                {3.783153, 34, 49, 4.79, 5.74},
                {4.010016, 34, 50, 4.01, 5.04},
                {4.185068, 34, 51.5, 2.94, 3.98},
                {4.395665, 34, 53, 5.49, 6.56},
                {4.595034, 34, 55.5, 4, 5.22},
                {4.794611, 34, 56, 0.59, 1.86},
                {4.994182, 34, 57, 3.67, 4.88},
                {5.134887, 34, 58, 4.60, 5.88},
                {5.332911, 34, 59, 4.31, 5.65},
                {5.598963, 34, 61, 1.24, 2.41},
                {5.878980, 34, 62, 0.4, 1.84},
                {6.081600, 34, 62, 1.64, 2.93}
        };

        // Data format: DISTANCE, FLYWHEEL_RPS, HOOD_ANGLE (Degrees), TOF (Start/End Time)
        for (double[] row : LUT_ROWS) {
            final double distance = row[0];

            DISTANCE_TO_HOOD_ANGLE.put(distance, Rotation2d.fromDegrees(row[1]));
            DISTANCE_TO_FLYWHEEL_RPS.put(distance, row[2]);
            DISTANCE_TO_TIME_OF_FLIGHT.put(distance, abs(row[4] - row[3]));
        }
    }

    public record ShootingParameters(boolean isValid,
                                     Rotation2d turretAngle, double turretVelocityRotPS,
                                     Rotation2d hoodAngle, double flywheelRPS) {
        public static final ShootingParameters INVALID = new ShootingParameters(false, kZero, 0, kZero, 0);
    }

    public ShootingParameters getResults() {
        if (latestParameters != null)
            return latestParameters;

        latestParameters = calculateShootingParameters();
        return latestParameters;
    }

    public double getMinTimeOfFlight() {
        return DISTANCE_TO_TIME_OF_FLIGHT.get(MIN_DISTANCE);
    }

    public double getMaxTimeOfFlight() {
        return DISTANCE_TO_TIME_OF_FLIGHT.get(MAX_DISTANCE);
    }

    public void invalidate() {
        latestParameters = null;
    }

    private ShootingParameters calculateShootingParameters() {
        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY_SECONDS);

        final Pose3d turretPose = new Pose3d(correctedPose).transformBy(ROBOT_TO_CENTER_TURRET);
        final Translation3d target = HUB_TOP_POSITION.get();

        final ChassisSpeeds robotSpeeds = SWERVE.getFieldRelativeVelocity();
        final Rotation2d robotHeading = correctedPose.getRotation();

        final double turretRelativeX = ROBOT_TO_CENTER_TURRET_2d.getX();
        final double turretRelativeY = ROBOT_TO_CENTER_TURRET_2d.getY();
        final double turretFieldX = turretRelativeX * robotHeading.getCos() - turretRelativeY * robotHeading.getSin();
        final double turretFieldY = turretRelativeX * robotHeading.getSin() + turretRelativeY * robotHeading.getCos();

        final double omega = robotSpeeds.omegaRadiansPerSecond;

        final double velocityX = robotSpeeds.vxMetersPerSecond - omega * turretFieldY;
        final double velocityY = robotSpeeds.vyMetersPerSecond + omega * turretFieldX;

        final double totalVelocity = Math.hypot(velocityX, velocityY);

        double predictedDistance = target.getDistance(turretPose.getTranslation());

        if (!isInRange(predictedDistance) || totalVelocity > MAX_SOTM_SPEED) {
            lastTurretAngle = null;
            return ShootingParameters.INVALID;
        }

        Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
        Rotation2d turretAngle = target.minus(turretPose.getTranslation()).toTranslation2d().getAngle();

        Pose3d hoodExitPosition = turretPose;
        Pose3d predictedExitPose = hoodExitPosition;

        Transform3d turretToHoodExit;
        int i = 0;

        double timeOfFlight = 0;

        if (totalVelocity < MIN_SOTM_SPEED) {
            turretToHoodExit = new Transform3d(
                    new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations()), 0, 0),
                    new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians()));

            hoodExitPosition = turretPose.transformBy(turretToHoodExit);
            predictedDistance = target.getDistance(hoodExitPosition.getTranslation());

            hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
            turretAngle = target.minus(hoodExitPosition.getTranslation()).toTranslation2d().getAngle();

            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);
        } else {
            for (; i < MAX_ITERATIONS; i++) {
                turretToHoodExit = new Transform3d(
                        new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations()), 0, 0),
                        new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians()));

                hoodExitPosition = turretPose.transformBy(turretToHoodExit);

                timeOfFlight = getDragCompensatedTimeOfFlight(DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance));

                final double offsetX = velocityX * timeOfFlight;
                final double offsetY = velocityY * timeOfFlight;

                predictedExitPose = new Pose3d(
                        new Translation3d(hoodExitPosition.getX() + offsetX, hoodExitPosition.getY() + offsetY, hoodExitPosition.getZ()),
                        hoodExitPosition.getRotation());

                final double newDistance = target.getDistance(predictedExitPose.getTranslation());
                final Rotation2d newHoodAngle = DISTANCE_TO_HOOD_ANGLE.get(newDistance);
                final Rotation2d newTurretAngle = target.minus(predictedExitPose.getTranslation()).toTranslation2d().getAngle();

                final boolean converged = abs(newDistance - predictedDistance) < DISTANCE_TOLERANCE_METERS &&
                        abs(newHoodAngle.minus(hoodAngle).getDegrees()) < HOOD_ANGLE_TOLERANCE_DEGREES &&
                        abs(newTurretAngle.minus(turretAngle).getRotations()) < TURRET_ANGLE_TOLERANCE_ROTATIONS;

                predictedDistance = newDistance;
                hoodAngle = newHoodAngle;
                turretAngle = newTurretAngle;

                if (converged) break;
            }

            if (Double.isNaN(timeOfFlight) || timeOfFlight <= 0) {
                invalidate();
                lastTurretAngle = null;
                return ShootingCalculator.ShootingParameters.INVALID;
            }
        }

        if (!isInRange(predictedDistance) || i >= MAX_ITERATIONS) {
            lastTurretAngle = null;
            return ShootingParameters.INVALID;
        }

        if (lastTurretAngle == null)
            lastTurretAngle = turretAngle;

        final double flywheelRPS = DISTANCE_TO_FLYWHEEL_RPS.get(predictedDistance);
        final double targetTurretVelocity = turretAngleFilter.calculate(turretAngle.minus(lastTurretAngle).getRotations() / PERIODIC_TIME_SEC);
        lastTurretAngle = turretAngle;

        final double confidence = computeConfidence(totalVelocity, predictedDistance, targetTurretVelocity);

        Logger.recordOutput("ShotCalculator/PredictedExitPose", predictedExitPose);
        Logger.recordOutput("ShotCalculator/TargetHoodAngle", hoodAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/TargetTurretAngle", turretAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/Distance", predictedDistance);
        Logger.recordOutput("ShotCalculator/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("ShotCalculator/TurretVelocityX", velocityX);
        Logger.recordOutput("ShotCalculator/TurretVelocityY", velocityY);
        Logger.recordOutput("ShotCalculator/Confidence", confidence);
        Logger.recordOutput("ShotCalculator/FlywheelRPS", flywheelRPS);

        return new ShootingParameters(
                confidence >= MIN_FIRE_CONFIDENCE,
                turretAngle,
                targetTurretVelocity,
                hoodAngle,
                flywheelRPS);
    }

    private double computeConfidence(double totalVelocity, double distance, double turretVelocityRPS) {
        final double distanceMargin = Math.min(distance - MIN_DISTANCE, MAX_DISTANCE - distance) / ((MAX_DISTANCE - MIN_DISTANCE) * 0.1);
        final double distanceQuality = MathUtil.clamp(distanceMargin, 0, 1);

        final double speedQuality = MathUtil.clamp(1.0 - totalVelocity / MAX_SOTM_SPEED, 0, 1);
        final double turretStability = MathUtil.clamp(1.0 - abs(turretVelocityRPS) / 1.0, 0, 1);

        return (distanceQuality * 0.50 + speedQuality * 0.30 + turretStability * 0.2) * 100.0;
    }

    private double getDragCompensatedTimeOfFlight(double timeOfFlight) {
        return (1 - Math.exp(-DRAG_K * timeOfFlight)) / DRAG_K;
    }

    private boolean isInRange(double distance) {
        return MIN_DISTANCE <= distance && distance <= MAX_DISTANCE;
    }
}
