package frc.robot.subsystems.shooter;

import static frc.robot.GlobalConstants.IS_SIMULATION;

public class ShootingConstants {
    public static final double PHASE_DELAY_SECONDS = IS_SIMULATION ? 0.0003 : (80 / 1000.0);
    public static final double MAX_DISTANCE = 5.195649;
    public static final double MIN_DISTANCE = 1.885479;

    public static final double MIN_FIRE_CONFIDENCE = 55.0;

    protected static final double DRAG_K = 0.47;

    protected static final double MIN_SOTM_SPEED = 0.1;
    protected static final double MAX_SOTM_SPEED = 3.5;

    protected static final int MAX_ITERATIONS = 15;
    protected static final double NEWTON_TOF_CONVERGENCE_TOLERANCE = 0.002;
}
