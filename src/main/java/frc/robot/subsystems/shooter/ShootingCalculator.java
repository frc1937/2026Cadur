package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

import static frc.lib.math.Conversions.toTransform2d;
import static frc.robot.GlobalConstants.PERIODIC_TIME_SEC;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_TURRET;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class ShootingCalculator {
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / PERIODIC_TIME_SEC));

    private double lastTurretAngle;
    private double lastHoodAngle;

    public static final double PHASE_DELAY = 0.045; //Total system latency. Commanded to shoot vs when the ball will exit.
    public static final double MIN_DISTANCE = 1.34;
    public static final double MAX_DISTANCE = 5.60;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null;

    //TODO: Tune these tables lmao
    static {
        DISTANCE_TO_FLYWHEEL_RPS.put(0.0, 0.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(2.0, 40.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(4.0, 55.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(6.0, 75.0);
        DISTANCE_TO_FLYWHEEL_RPS.put(8.0, 95.0);

        DISTANCE_TO_HOOD_ANGLE.put(0.0, 0.0);
        DISTANCE_TO_HOOD_ANGLE.put(2.0, 0.20);
        DISTANCE_TO_HOOD_ANGLE.put(4.0, 0.15);
        DISTANCE_TO_HOOD_ANGLE.put(6.0, 0.12);
        DISTANCE_TO_HOOD_ANGLE.put(8.0, 0.10);

        DISTANCE_TO_TIME_OF_FLIGHT.put(0.0, 0.0);
        DISTANCE_TO_TIME_OF_FLIGHT.put(2.0, 0.10);
        DISTANCE_TO_TIME_OF_FLIGHT.put(4.0, 0.22);
        DISTANCE_TO_TIME_OF_FLIGHT.put(6.0, 0.35);
        DISTANCE_TO_TIME_OF_FLIGHT.put(9.0, 0.55);
    }

    public record ShootingParameters(boolean isValid, double turretAngle, double turretVelocity, double hoodAngle,
                                     double hoodVelocity,
                                     double flywheelRPS) {
    }


    public ShootingParameters calculateParameters() {
        if (latestParameters != null) {
            return latestParameters;
        }
        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY);

        final var target = HUB_TOP_POSITION.get().toTranslation2d();
        final var turretPosition = correctedPose.transformBy(toTransform2d(ROBOT_TO_TURRET));

        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        ChassisSpeeds robotSpeeds = SWERVE.getFieldRelativeVelocity();

        double robotAngle = correctedPose.getRotation().getRadians();
        double turretVelocityX = robotSpeeds.vxMetersPerSecond
                + robotSpeeds.omegaRadiansPerSecond
                * (ROBOT_TO_TURRET.getY() * Math.cos(robotAngle)
                - ROBOT_TO_TURRET.getX() * Math.sin(robotAngle));
        double turretVelocityY = robotSpeeds.vyMetersPerSecond
                + robotSpeeds.omegaRadiansPerSecond
                * (ROBOT_TO_TURRET.getX() * Math.cos(robotAngle)
                - ROBOT_TO_TURRET.getY() * Math.sin(robotAngle));

        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT.get(lookaheadTurretToTargetDistance);

            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;

            lookaheadPose = new Pose2d(
                            turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                            turretPosition.getRotation());

            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        double turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle().getRotations();
        double hoodAngle = DISTANCE_TO_HOOD_ANGLE.get(lookaheadTurretToTargetDistance);

        if (Double.isNaN(lastTurretAngle)) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

        double turretVelocity = turretAngleFilter.calculate((turretAngle - lastTurretAngle) / PERIODIC_TIME_SEC);
        double hoodVelocity = hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / PERIODIC_TIME_SEC);
        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;

        latestParameters = new ShootingParameters(
                lookaheadTurretToTargetDistance >= MIN_DISTANCE && lookaheadTurretToTargetDistance <= MAX_DISTANCE,
                turretAngle,
                turretVelocity,
                hoodAngle,
                hoodVelocity,
                DISTANCE_TO_FLYWHEEL_RPS.get(lookaheadTurretToTargetDistance));

        Logger.recordOutput("ShotCalculator/LookaheadPose", lookaheadPose);
        Logger.recordOutput("ShotCalculator/TurretToTargetDistance", lookaheadTurretToTargetDistance);

        return latestParameters;
    }

    public void clearLatestParameters() {
        latestParameters = null;
    }
}
