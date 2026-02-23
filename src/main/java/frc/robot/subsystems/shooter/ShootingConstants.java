package frc.robot.subsystems.shooter;

import static frc.robot.GlobalConstants.IS_SIMULATION;

public class ShootingConstants {
    public static final double PHASE_DELAY = IS_SIMULATION ? 0.0003 : 0.03; //TODO TUNE Total system latency. Commanded to shoot vs when the ball will exit.
    public static final double MAX_DISTANCE = 5.60;
    public static final double MIN_DISTANCE = 1.34;


    protected static final double DISTANCE_TOLERANCE_METERS = 0.001;
    protected static final double HOOD_ANGLE_TOLERANCE_DEGREES = 0.1;
    protected static final int MAX_ITERATIONS = 10;
}
