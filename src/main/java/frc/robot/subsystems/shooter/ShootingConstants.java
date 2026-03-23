package frc.robot.subsystems.shooter;

import static frc.robot.GlobalConstants.IS_SIMULATION;

public class ShootingConstants {
    public static final double PHASE_DELAY_SECONDS = IS_SIMULATION ? 0.0003 : (85 / 1000.0);
    public static final double MAX_DISTANCE = 6.09000;
    public static final double MIN_DISTANCE = 1.77;

    public static final double MIN_FIRE_CONFIDENCE = 55.0;

    protected static final double DRAG_K = 0.36;

    protected static final double MIN_SOTM_SPEED = 0.1;
    protected static final double MAX_SOTM_SPEED = 3.5;

    protected static final int MAX_ITERATIONS = 15;

    protected static final double
            TURRET_ANGLE_TOLERANCE_ROTATIONS = 0.001,
            HOOD_ANGLE_TOLERANCE_DEGREES = 0.01,
            DISTANCE_TOLERANCE_METERS = 0.001;
}
