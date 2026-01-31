package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.lib.math.Conversions.toTransform2d;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.shooter.turret.TurretConstants.ROBOT_TO_TURRET;
import static frc.robot.utilities.FieldConstants.HUB_TOP_POSITION;

public class ShootingCalculator {
    public static final double PHASE_DELAY = 0.045; //Total system latency. Commanded to shoot vs when the ball will exit.

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_FLYWHEEL_RPS = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_ANGLE = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

    private static ShootingParameters latestParameters = null; //Ensure we only calculate once per cycle

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

    public record ShootingParameters(double turretAngle, double turretVelocity, double hoodAngle, double hoodVelocity, double flywheelRPS) {
    }

    /*
┌─────────────────────────────────────────────────────────────┐
│ Current Robot Pose + Velocity                               │
│   ↓ (Apply phase delay)                                     │
│ Future Robot Pose                  V                        │
│   ↓ (Calculate distance to target)                          │
│ Turret-to-Target Distance                                   │
│   ↓ (Account for robot movement during flight)              │
│ LOOKAHEAD: Iterative prediction (up to 20x)                 │
│   - Estimate time of flight                                 │
│   - Calculate where target will be when note arrives        │
│   - Adjust aim point                                        │
│   ↓                                                         │
│ Final Shooting Parameters                                   │
└─────────────────────────────────────────────────────────────┘
     */

    public ShootingParameters calculateParameters() {
        if (latestParameters != null) {
            return latestParameters;
        } //TODO: Go over MA-6328 code and implement the missing parts

        final Pose2d correctedPose = POSE_ESTIMATOR.predictFuturePose(PHASE_DELAY);

        final var turretPosition = correctedPose.transformBy(toTransform2d(ROBOT_TO_TURRET));
        final var target = HUB_TOP_POSITION.get().toTranslation2d();

        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        return latestParameters;
    }

    public void clearLatestParameters() {
        latestParameters = null;
    }
}
