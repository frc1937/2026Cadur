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

public class ShootingCalculator {
    private static final double DERIVATIVE_H = 0.01;

    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));
    private Rotation2d lastTurretAngle;

    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;
    private double latestCalculationTimestamp = -1;

    private double previousTimeOfFlight = -1;

    // TODO NEW TABLE (DIST, FLY, HOOD, TOF start/end(may be inverted))
    //1.885479, 33, 18, 1.82, 2.8
    //2.118483, 34.5, 19, 2.50, 3.53
    //2.314063, 35.5, 20.5, 1.79, 2.79
    //2.506295, 36.5, 28.5, 2.18, 3.19
    //2.699365, 36.5, 30.500000, 2.50, 3.45
    //2.918604, 36.5, 33.5, 5.03, 5.96
    //3.03, 36.5, 35.5, 2.60, 3.48
    //3.225192, 36.5, 37.5, 4.29, 5.15
    //3.440166, 39.5, 37.5, 2.30, 3.26
    //3.680015, 43.5, 34.5 3.30, 4.48
    //3.930581, 45, 35.5, 2.34, 3.50
    //4.177848, 46.000000, 35.5, 5.72, 6.91
    //4.418539, 45, 38,  2.37, 3.37
    //4.623432, 47.5, 40, 0.90, 1.72
    //4.858302, 48.5, 40, 2.58, 3.70
    //5.022761, 49, 40, 3.10, 2.02
    //5.195649, 50, 40, 5.27, 6.40

    static {
        // Data format: DISTANCE, FLYWHEEL_RPS, HOOD_ANGLE (Degrees), TOF (End Time)

        // 1.885479, 33, 18, 1.82, 2.8
        DISTANCE_TO_FLYWHEEL_RPS.put(1.885479, 33.0);
        DISTANCE_TO_HOOD_ANGLE.put(1.885479, Rotation2d.fromDegrees(18.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.885479, 2.8 - 1.82);

        // 2.118483, 34.5, 19, 2.50, 3.53
        DISTANCE_TO_FLYWHEEL_RPS.put(2.118483, 34.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.118483, Rotation2d.fromDegrees(19.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.118483, 1.03);

        // 2.314063, 35.5, 20.5, 1.79, 2.79
        DISTANCE_TO_FLYWHEEL_RPS.put(2.314063, 35.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.314063, Rotation2d.fromDegrees(20.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.314063, 1.0);

        // 2.506295, 36.5, 28.5, 2.18, 3.19
        DISTANCE_TO_FLYWHEEL_RPS.put(2.506295, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.506295, Rotation2d.fromDegrees(28.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.506295, 3.19 - 2.18);

        // 2.699365, 36.5, 30.5, 2.50, 3.45
        DISTANCE_TO_FLYWHEEL_RPS.put(2.699365, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.699365, Rotation2d.fromDegrees(30.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.699365, 3.45 - 2.5);

        // 2.918604, 36.5, 33.5, 5.03, 5.96
        DISTANCE_TO_FLYWHEEL_RPS.put(2.918604, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.918604, Rotation2d.fromDegrees(33.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.918604, 5.96 - 5.03);

        // 3.03, 36.5, 35.5, 2.60, 3.48
        DISTANCE_TO_FLYWHEEL_RPS.put(3.03, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.03, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.03, 3.48 - 2.6);

        // 3.225192, 36.5, 37.5, 4.29, 5.15
        DISTANCE_TO_FLYWHEEL_RPS.put(3.225192, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.225192, Rotation2d.fromDegrees(37.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.225192, 5.15 - 4.29);

        // 3.440166, 39.5, 37.5, 2.30, 3.26
        DISTANCE_TO_FLYWHEEL_RPS.put(3.440166, 39.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.440166, Rotation2d.fromDegrees(37.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.440166, 3.26 - 2.30);

        // 3.680015, 43.5, 34.5, 3.30, 4.48
        DISTANCE_TO_FLYWHEEL_RPS.put(3.680015, 43.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.680015, Rotation2d.fromDegrees(34.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.680015, 4.48 - 3.30);

        // 3.930581, 45, 35.5, 2.34, 3.50
        DISTANCE_TO_FLYWHEEL_RPS.put(3.930581, 45.0);
        DISTANCE_TO_HOOD_ANGLE.put(3.930581, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.930581, 3.50 - 2.34);

        // 4.177848, 46.0, 35.5, 5.72, 6.91
        DISTANCE_TO_FLYWHEEL_RPS.put(4.177848, 46.0);
        DISTANCE_TO_HOOD_ANGLE.put(4.177848, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.177848, 6.91 - 5.72);

        // 4.418539, 45, 38,  2.37, 3.37
        DISTANCE_TO_FLYWHEEL_RPS.put(4.418539, 45.0);
        DISTANCE_TO_HOOD_ANGLE.put(4.418539, Rotation2d.fromDegrees(38.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.418539, 1.0);

        // 4.623432, 47.5, 40, 0.90, 1.72
        DISTANCE_TO_FLYWHEEL_RPS.put(4.623432, 47.5);
        DISTANCE_TO_HOOD_ANGLE.put(4.623432, Rotation2d.fromDegrees(40.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.623432, 1.72 - 0.9);

        // 4.858302, 48.5, 40, 2.58, 3.70
        DISTANCE_TO_FLYWHEEL_RPS.put(4.858302, 48.5);
        DISTANCE_TO_HOOD_ANGLE.put(4.858302, Rotation2d.fromDegrees(40.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.858302, 1.12);

        // 5.022761, 49, 40, 3.10, 2.02 -> INVERTED (Used 3.10 as the 'hit' time)
        DISTANCE_TO_FLYWHEEL_RPS.put(5.022761, 49.0);
        DISTANCE_TO_HOOD_ANGLE.put(5.022761, Rotation2d.fromDegrees(40.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(5.022761, 1.08);

        // 5.195649, 50, 40, 5.27, 6.40
        DISTANCE_TO_FLYWHEEL_RPS.put(5.195649, 50.0);
        DISTANCE_TO_HOOD_ANGLE.put(5.195649, Rotation2d.fromDegrees(40.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(5.195649, 1.13);
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
        previousTimeOfFlight = -1.0;
    }
    
    private ShootingParameters calculateShootingParameters() {
        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY_SECONDS);

        final Pose3d turretPosition = new Pose3d(correctedPose).transformBy(ROBOT_TO_CENTER_TURRET);
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

        if (totalVelocity > MAX_SOTM_SPEED)
            return ShootingParameters.INVALID;

        double predictedDistance = target.getDistance(turretPosition.getTranslation());

        if (!isInRange(predictedDistance))
            return ShootingParameters.INVALID;

        Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
        Rotation2d turretAngle = target.minus(turretPosition.getTranslation()).toTranslation2d().getAngle();

        Pose3d hoodExitPosition = turretPosition;
        Pose3d predictedExitPose = hoodExitPosition;

        Transform3d turretToHoodExit;
        int i = 0;

        double timeOfFlight = 0;

        if (totalVelocity < MIN_SOTM_SPEED) {
            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);
        } else {
//            for (; i < MAX_ITERATIONS; i++) {
//                turretToHoodExit = new Transform3d(
//                        new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations()), 0, 0),
//                        new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians()));
//
//                hoodExitPosition = turretPosition.transformBy(turretToHoodExit);
//
//                timeOfFlight = getDragCompensatedTimeOfFlight(DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance));
//
//                final double offsetX = velocityX * timeOfFlight;
//                final double offsetY = velocityY * timeOfFlight;
//
//                predictedExitPose = new Pose3d(
//                        new Translation3d(hoodExitPosition.getX() + offsetX, hoodExitPosition.getY() + offsetY, hoodExitPosition.getZ()),
//                        hoodExitPosition.getRotation());
//
//                final double newDistance = target.getDistance(predictedExitPose.getTranslation());
//                final Rotation2d newHoodAngle = DISTANCE_TO_HOOD_ANGLE.get(newDistance);
//                final Rotation2d newTurretAngle = target.minus(predictedExitPose.getTranslation()).toTranslation2d().getAngle();
//
//                final boolean converged = abs(newDistance - predictedDistance) < DISTANCE_TOLERANCE_METERS &&
//                        abs(newHoodAngle.minus(hoodAngle).getDegrees()) < HOOD_ANGLE_TOLERANCE_DEGREES &&
//                        abs(newTurretAngle.minus(turretAngle).getRotations()) < TURRET_ANGLE_TOLERANCE_ROTATIONS;
//
//                predictedDistance = newDistance;
//                hoodAngle = newHoodAngle;
//                turretAngle = newTurretAngle;
//
//                if (converged) break;
//            }

            timeOfFlight = (previousTimeOfFlight > 0) ? previousTimeOfFlight : DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);

            for (; i < MAX_ITERATIONS; i++) {
                turretToHoodExit = new Transform3d(
                        new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations()), 0, 0),
                        new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians()));

                hoodExitPosition = turretPosition.transformBy(turretToHoodExit);

                double effectiveTimeOfFlight = getDragCompensatedTimeOfFlight(timeOfFlight);

                double projX = hoodExitPosition.getX() + velocityX * effectiveTimeOfFlight;
                double projY = hoodExitPosition.getY() + velocityY * effectiveTimeOfFlight;
                double projZ = hoodExitPosition.getZ();

                double rx = target.getX() - projX;
                double ry = target.getY() - projY;
                double rz = target.getZ() - projZ;
                predictedDistance = Math.sqrt(rx * rx + ry * ry + rz * rz);

                double lookupTimeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance);

                double dTdd = timeOfFlightDerivative(predictedDistance);
                double rDotV = -(rx * velocityX + ry * velocityY) / predictedDistance;
                double dEffDt = Math.exp(-DRAG_K * timeOfFlight);
                double fPrime = dTdd * rDotV * dEffDt - 1.0;

                double prevTimeOfFlight = timeOfFlight;
                if (Math.abs(fPrime) > 1e-3) {
                    timeOfFlight = timeOfFlight - (lookupTimeOfFlight - timeOfFlight) / fPrime;
                } else {
                    timeOfFlight = lookupTimeOfFlight;
                }

                // Per-iteration clamp prevents runaway
                timeOfFlight = MathUtil.clamp(timeOfFlight, 0.05, 5.0);

                // Update angle estimates for next iteration
                hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
                turretAngle = new Translation2d(rx, ry).getAngle();

                if (Math.abs(timeOfFlight - prevTimeOfFlight) < NEWTON_TOF_CONVERGENCE_TOLERANCE) {
                    i = i + 1;
                    break;
                }
            }

            if (Double.isNaN(timeOfFlight) || timeOfFlight <= 0) {
                invalidate();
                return ShootingCalculator.ShootingParameters.INVALID;
            }
        }

        previousTimeOfFlight = timeOfFlight;

        if (!isInRange(predictedDistance) || i >= MAX_ITERATIONS)
            return ShootingParameters.INVALID;

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
        final double turretStability = MathUtil.clamp(1.0 - Math.abs(turretVelocityRPS) / 1, 0, 1);

        return (distanceQuality*0.55 + speedQuality*0.35 + turretStability*0.2) * 100.0;
    }

    private double getDragCompensatedTimeOfFlight(double timeOfFlight) {
        return (1 - Math.exp(-DRAG_K * timeOfFlight)) / DRAG_K;
    }

    private boolean isInRange(double distance) {
        return MIN_DISTANCE <= distance && distance <= MAX_DISTANCE;
    }

    /**
     * Central finite difference derivative of the TOF LUT.
     * Used in the Newton step. Much more accurate than analytic approximation when LUT is noisy.
     */
    private static double timeOfFlightDerivative(double distanceM) {
        return (DISTANCE_TO_TIME_OF_FLIGHT.get(distanceM + DERIVATIVE_H) - DISTANCE_TO_TIME_OF_FLIGHT.get(distanceM - DERIVATIVE_H)) / (2.0 * DERIVATIVE_H);
    }
}
