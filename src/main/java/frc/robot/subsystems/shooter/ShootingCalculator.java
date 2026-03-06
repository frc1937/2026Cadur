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

    private static final double DRAG_K = 0.5; //TODO TUne

    //TODO TABLE: DIST, FLY, HOOD DEG, TOF
    // 1.67, 28. 22.5, 0.76
    // 1.93, 30, 22, 0.85
    // 2.213, 33, 25, 0.92
    // 2.47, 33, 24, 0.84
    // 2.77, 34, 26.5, 1.00
    // 3.06, 38, 30, 0.82
    // 3.31, 38, 31.5, 1.01 ADD THIS TOO, FURTHER TUNING REQUIRED!!! TUNE CAMERAS FOR 1600 x 1200
    //

    static {
        DISTANCE_TO_FLYWHEEL_RPS.put(1.67, 28.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(1.93, 30.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.213, 33.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.47, 33.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.77, 34.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(3.06, 38.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(3.31, 38.0);

        DISTANCE_TO_HOOD_ANGLE.put(1.67, Rotation2d.fromDegrees(22.5));
        DISTANCE_TO_HOOD_ANGLE.put(1.93, Rotation2d.fromDegrees(22.0));
        DISTANCE_TO_HOOD_ANGLE.put(2.213, Rotation2d.fromDegrees(25.0));
        DISTANCE_TO_HOOD_ANGLE.put(2.47, Rotation2d.fromDegrees(24.0));
        DISTANCE_TO_HOOD_ANGLE.put(2.77, Rotation2d.fromDegrees(26.5));
        DISTANCE_TO_HOOD_ANGLE.put(3.06, Rotation2d.fromDegrees(30.0));
        DISTANCE_TO_HOOD_ANGLE.put(3.31, Rotation2d.fromDegrees(31.5));

        DISTANCE_TO_TIME_OF_FLIGHT.put(1.67, 0.76);
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.93, 0.85);
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.213, 0.92);
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.47, 0.84);
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.77, 1.00);
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.06, 0.82);
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.31, 1.01);
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

        double turretRelativeX = ROBOT_TO_CENTER_TURRET.getX();
        double turretRelativeY = ROBOT_TO_CENTER_TURRET.getY();

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
