package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

    //----------------------------------------
    private static final String KEY = "HubShotTuner/";
    private final static LoggedNetworkNumber hoodAngleDegrees = new LoggedNetworkNumber(KEY + "hoodAngleDeg", 14.0);
    private final static LoggedNetworkNumber flywheelSpeedRPS = new LoggedNetworkNumber(KEY + "flywheelSpeedRPS", 30.0);
    //----------------------------------------

    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;

    static {
        final double[][] LUT_ROWS = {
                {1.8070034444007221, 14, 43, 1.99, 2.99},
                {2.0402865607842444, 18, 41, 4.74, 5.66},
                {2.19057628328192, 22, 41, 0.75, 1.63},
                {2.40030449760978, 27, 41, 4.11, 4.93},
                {2.598391421160679, 29, 43, 0.46, 1.31},
                {2.806797, 31.5, 43.5, 0.09, 0.96},
                {3.019959, 33, 44, 0.78, 1.63},
                {3.194163, 34, 44.5, 1.16, 2.02},
                {3.396257, 34, 46.1, 2.2, 3.16},
                {3.598324, 34, 48, 1.46, 2.43},
                {3.783153, 34, 49, 4.79, 5.74},
                {4.010016, 34, 50, 4.01, 5.04},
                {4.185068, 34, 51.5, 2.94, 3.98},
                {4.395665, 34, 53, 5.49, 6.56},
                {4.595034, 34, 55.5, 4, 5.22},
                {4.794611, 34, 56, 0.59, 1.86},
                {4.994182, 34, 57, 3.67, 4.88},
                {5.134887, 34, 58, 4.60, 5.88},
                {5.332911, 34, 59, 4.31, 5.65},
                {5.613281, 34, 60.5, 1.61, 3},
                {5.935968, 34, 63, 3.23, 4.57}
        };

        // Data format: DISTANCE, HOOD_ANGLE (Degrees), FLYWHEEL_RPS, TOF (Start/End Time)
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

    private static Translation3d computeHoodExitTranslation(
            Translation3d turretPivot,
            Rotation2d turretAngleField,
            Rotation2d hoodAngle,
            double barrelLength) {

        final double cosHood = Math.cos(hoodAngle.getRadians());
        final double sinHood = Math.sin(hoodAngle.getRadians());

        final double dx = barrelLength * cosHood * turretAngleField.getCos();
        final double dy = barrelLength * cosHood * turretAngleField.getSin();

        final double dz = barrelLength * sinHood;

        return turretPivot.plus(new Translation3d(dx, dy, dz));
    }

    private ShootingParameters calculateShootingParameters() {
        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY_SECONDS);

        final Pose3d turretPose = new Pose3d(correctedPose).transformBy(ROBOT_TO_CENTER_TURRET);
        final Translation3d turretPivot = turretPose.getTranslation();
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

        double predictedDistance = target.getDistance(turretPivot);

        Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
        Rotation2d turretAngle = target.minus(turretPivot).toTranslation2d().getAngle();

        Translation3d hoodExitTranslation = turretPivot;
        Pose3d predictedExitPose = turretPose;

        int i = 0;
        double timeOfFlight = 0;

        if (totalVelocity < MIN_SOTM_SPEED) {
            // Stationary: one refinement pass with barrel offset
            final double barrelLength = HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations());
            hoodExitTranslation = computeHoodExitTranslation(turretPivot, turretAngle, hoodAngle, barrelLength);

            predictedDistance = target.getDistance(hoodExitTranslation);
            hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
            turretAngle = target.minus(hoodExitTranslation).toTranslation2d().getAngle();

            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);

            predictedExitPose = new Pose3d(hoodExitTranslation, turretPose.getRotation());
        } else {
            for (; i < MAX_ITERATIONS; i++) {
                final double barrelLength = HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations());
                hoodExitTranslation = computeHoodExitTranslation(turretPivot, turretAngle, hoodAngle, barrelLength);

                timeOfFlight = getDragCompensatedTimeOfFlight(DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance));

                final double offsetX = velocityX * timeOfFlight;
                final double offsetY = velocityY * timeOfFlight;

                final Translation3d predictedExitTranslation = new Translation3d(
                        hoodExitTranslation.getX() + offsetX,
                        hoodExitTranslation.getY() + offsetY,
                        hoodExitTranslation.getZ());

                predictedExitPose = new Pose3d(predictedExitTranslation, turretPose.getRotation());

                final double newDistance = target.getDistance(predictedExitTranslation);
                final Rotation2d newHoodAngle = DISTANCE_TO_HOOD_ANGLE.get(newDistance);
                final Rotation2d newTurretAngle = target.minus(predictedExitTranslation).toTranslation2d().getAngle();

                final boolean converged = abs(newDistance - predictedDistance) < DISTANCE_TOLERANCE_METERS &&
                        abs(newHoodAngle.minus(hoodAngle).getDegrees()) < HOOD_ANGLE_TOLERANCE_DEGREES &&
                        abs(newTurretAngle.minus(turretAngle).getRotations()) < TURRET_ANGLE_TOLERANCE_ROTATIONS;

                predictedDistance = newDistance;
                hoodAngle = newHoodAngle;
                turretAngle = newTurretAngle;

                if (converged) break;
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
//        return new ShootingParameters(true,
//                turretAngle,
//                targetTurretVelocity,
//                Rotation2d.fromDegrees(hoodAngleDegrees.get()),
//                flywheelSpeedRPS.get()
//        );
    }

    private double computeConfidence(double totalVelocity, double distance, double turretVelocityRPS) {
        final double distanceMargin = Math.min(distance - MIN_DISTANCE, MAX_DISTANCE - distance) / ((MAX_DISTANCE - MIN_DISTANCE) * 0.1);
        final double distanceQuality = MathUtil.clamp(distanceMargin, 0, 1);

        final double speedQuality = MathUtil.clamp(1.0 - totalVelocity / MAX_SOTM_SPEED, 0, 1);
        final double turretStability = MathUtil.clamp(1.0 - abs(turretVelocityRPS) / 1.0, 0, 1);

        return (distanceQuality * 0.30 + speedQuality * 0.40 + turretStability * 0.3) * 100.0;
    }

    private double getDragCompensatedTimeOfFlight(double timeOfFlight) {
        return (1 - Math.exp(-DRAG_K * timeOfFlight)) / DRAG_K;
    }

    private boolean isInRange(double distance) {
        return MIN_DISTANCE <= distance;// && distance <= MAX_DISTANCE;
    }
}