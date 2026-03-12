package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.interpolation.InverseInterpolator.forDouble;
import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.ShootingConstants.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.HOOD_ANGLE_TO_SHOOTER_LENGTH;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class ShootingCalculator {
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));
    private Rotation2d lastTurretAngle;

    public static final InterpolatingTreeMap<Double, Rotation2d> DISTANCE_TO_HOOD_ANGLE = new InterpolatingTreeMap<>(forDouble(), Rotation2d::interpolate);
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;

    private static final double DRAG_K = 0.385;

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
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.885479, 2.8-1.82);

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
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.506295, 3.19-2.18);

        // 2.699365, 36.5, 30.5, 2.50, 3.45
        DISTANCE_TO_FLYWHEEL_RPS.put(2.699365, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.699365, Rotation2d.fromDegrees(30.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.699365, 3.45-2.5);

        // 2.918604, 36.5, 33.5, 5.03, 5.96
        DISTANCE_TO_FLYWHEEL_RPS.put(2.918604, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(2.918604, Rotation2d.fromDegrees(33.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.918604, 5.96-5.03);

        // 3.03, 36.5, 35.5, 2.60, 3.48
        DISTANCE_TO_FLYWHEEL_RPS.put(3.03, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.03, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.03, 3.48-2.6);

        // 3.225192, 36.5, 37.5, 4.29, 5.15
        DISTANCE_TO_FLYWHEEL_RPS.put(3.225192, 36.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.225192, Rotation2d.fromDegrees(37.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.225192, 5.15-4.29);

        // 3.440166, 39.5, 37.5, 2.30, 3.26
        DISTANCE_TO_FLYWHEEL_RPS.put(3.440166, 39.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.440166, Rotation2d.fromDegrees(37.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.440166, 3.26-2.30);

        // 3.680015, 43.5, 34.5, 3.30, 4.48
        DISTANCE_TO_FLYWHEEL_RPS.put(3.680015, 43.5);
        DISTANCE_TO_HOOD_ANGLE.put(3.680015, Rotation2d.fromDegrees(34.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.680015, 4.48-3.30);

        // 3.930581, 45, 35.5, 2.34, 3.50
        DISTANCE_TO_FLYWHEEL_RPS.put(3.930581, 45.0);
        DISTANCE_TO_HOOD_ANGLE.put(3.930581, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.930581, 3.50-2.34);

        // 4.177848, 46.0, 35.5, 5.72, 6.91
        DISTANCE_TO_FLYWHEEL_RPS.put(4.177848, 46.0);
        DISTANCE_TO_HOOD_ANGLE.put(4.177848, Rotation2d.fromDegrees(35.5));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.177848, 6.91-5.72);

        // 4.418539, 45, 38,  2.37, 3.37
        DISTANCE_TO_FLYWHEEL_RPS.put(4.418539, 45.0);
        DISTANCE_TO_HOOD_ANGLE.put(4.418539, Rotation2d.fromDegrees(38.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.418539, 1.0);

        // 4.623432, 47.5, 40, 0.90, 1.72
        DISTANCE_TO_FLYWHEEL_RPS.put(4.623432, 47.5);
        DISTANCE_TO_HOOD_ANGLE.put(4.623432, Rotation2d.fromDegrees(40.0));
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.623432, 1.72-0.9);

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

    public record ShootingParameters(boolean isValid, Rotation2d turretAngle, double turretVelocityRotPS,
                                     Rotation2d hoodAngle,
                                     double flywheelRPS) {
    }

    public ShootingParameters getResults() {
        if (latestParameters != null) return latestParameters;

        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY);

        final var turretPosition = new Pose3d(correctedPose).transformBy(ROBOT_TO_CENTER_TURRET);
        final var target = HUB_TOP_POSITION.get();

        var hoodExitPosition = turretPosition;

        ChassisSpeeds robotSpeeds = SWERVE.getFieldRelativeVelocity();
        Rotation2d robotHeading = correctedPose.getRotation();

        double turretRelativeX = ROBOT_TO_CENTER_TURRET_2d.getX();
        double turretRelativeY = ROBOT_TO_CENTER_TURRET_2d.getY();

        double turretFieldX = turretRelativeX * robotHeading.getCos() - turretRelativeY * robotHeading.getSin();
        double turretFieldY = turretRelativeX * robotHeading.getSin() + turretRelativeY * robotHeading.getCos();

        double tangentialVelocityX = -(robotSpeeds.omegaRadiansPerSecond) * turretFieldY;
        double tangentialVelocityY = (robotSpeeds.omegaRadiansPerSecond) * turretFieldX;

        double turretVelocityX = robotSpeeds.vxMetersPerSecond + tangentialVelocityX;
        double turretVelocityY = robotSpeeds.vyMetersPerSecond + tangentialVelocityY;

        double timeOfFlight = 0;
        double predictedDistance = target.getDistance(turretPosition.getTranslation());

        Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(predictedDistance);
        Rotation2d turretAngle = target.minus(turretPosition.getTranslation()).toTranslation2d().getAngle();

        Pose3d predictedExitPose = hoodExitPosition;

        Transform3d turretToHoodExit;
        int i;

        for (i = 0; i < MAX_ITERATIONS; i++) {
            turretToHoodExit = new Transform3d(
                    new Translation3d(HOOD_ANGLE_TO_SHOOTER_LENGTH.get(hoodAngle.getRotations()), 0, 0),
                    new Rotation3d(0, hoodAngle.getRadians(), turretAngle.getRadians())
            );

            hoodExitPosition = turretPosition.transformBy(turretToHoodExit);

            timeOfFlight = getDragCompensatedTimeOfFlight(DISTANCE_TO_TIME_OF_FLIGHT.get(predictedDistance));

            final double offsetX = turretVelocityX * timeOfFlight;
            final double offsetY = turretVelocityY * timeOfFlight;

            predictedExitPose = new Pose3d(
                    new Translation3d(hoodExitPosition.getX() + offsetX, hoodExitPosition.getY() + offsetY, hoodExitPosition.getZ()),
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
        Logger.recordOutput("ShotCalculator/InRange", inRange);

        return latestParameters;
    }

    public void clearLatestParameters() {
        latestParameters = null;
    }

    public double getMinTimeOfFlight() {
        return DISTANCE_TO_TIME_OF_FLIGHT.get(MIN_DISTANCE);
    }

    public double getMaxTimeOfFlight() {
        return DISTANCE_TO_TIME_OF_FLIGHT.get(MAX_DISTANCE);
    }

    private double getDragCompensatedTimeOfFlight(double timeOfFlight) {
        return (1 - Math.exp(-DRAG_K * timeOfFlight)) / DRAG_K;
    }
}
