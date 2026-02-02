package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.interpolation.InverseInterpolator.forDouble;
import static frc.lib.math.Conversions.toTransform2d;
import static frc.robot.GlobalConstants.IS_SIMULATION;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_TURRET;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class ShootingCalculator {
    public static final double PHASE_DELAY = IS_SIMULATION ? 0.003 : 0.03; //Total system latency. Commanded to shoot vs when the ball will exit.
    public static final double MIN_DISTANCE = 1.34;
    public static final double MAX_DISTANCE = 5.60;

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

        DISTANCE_TO_TIME_OF_FLIGHT.put(1.38, 0.90);
        DISTANCE_TO_TIME_OF_FLIGHT.put(1.88, 1.09);
        DISTANCE_TO_TIME_OF_FLIGHT.put(3.15, 1.11);
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.55, 1.12);
        DISTANCE_TO_TIME_OF_FLIGHT.put(5.68, 1.16);
    }

    public record ShootingParameters(boolean isValid, Rotation2d turretAngle, Rotation2d hoodAngle, double flywheelRPS) {
    }

    public ShootingParameters getParameters() {
        if (latestParameters != null) return latestParameters;

        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY);

        final var target = HUB_TOP_POSITION.get().toTranslation2d();
        final var turretPosition = correctedPose.transformBy(toTransform2d(ROBOT_TO_TURRET));
        final var hoodExitPosition = turretPosition; // TODO: use exit location

        double turretToTargetDistance = target.getDistance(hoodExitPosition.getTranslation());

        ChassisSpeeds robotSpeeds = SWERVE.getFieldRelativeVelocity();

        double turretVelocityX = robotSpeeds.vxMetersPerSecond
                + robotSpeeds.omegaRadiansPerSecond
                * (ROBOT_TO_TURRET.getY() * correctedPose.getRotation().getCos()
                - ROBOT_TO_TURRET.getX() * correctedPose.getRotation().getSin());

        double turretVelocityY = robotSpeeds.vyMetersPerSecond
                + robotSpeeds.omegaRadiansPerSecond
                * (ROBOT_TO_TURRET.getX() * correctedPose.getRotation().getCos()
                - ROBOT_TO_TURRET.getY() * correctedPose.getRotation().getSin());

        Pose2d lookaheadPose = hoodExitPosition;
        double timeOfFlight;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(lookaheadTurretToTargetDistance);

            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;

            lookaheadPose = new Pose2d(
                    hoodExitPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    hoodExitPosition.getRotation());

            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        final Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
        final Rotation2d hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(lookaheadTurretToTargetDistance);

        latestParameters = new ShootingParameters(
                lookaheadTurretToTargetDistance >= MIN_DISTANCE && lookaheadTurretToTargetDistance <= MAX_DISTANCE,
                turretAngle,
                hoodAngle,
                DISTANCE_TO_FLYWHEEL_RPS.get(lookaheadTurretToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TargetHoodAngle", hoodAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/TargetTurretAngle", turretAngle.getDegrees());
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestParameters;
    }

    public void clearLatestParameters() {
        latestParameters = null;
    }
}
