package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimulatedBall {
    private Pose3d pose;
    private Translation3d vel;
    private double lifetime = 0;
    private boolean active = true;
    private double spinRadiansPerSecond;

    /** Axis about which the ball spins (unit vector in field frame). */
    private final Translation3d spinAxis;

    // Physics constants
    private static final double AIR_DENSITY = 1.205;
    private static final double DRAG_COEFFICIENT = 0.5;
    private static final double GAME_PIECE_MASS_KG = 0.21;
    private static final double GAME_PIECE_AREA = Math.PI * 0.075 * 0.075;
    private static final double MAGNUS_LIFT_FACTOR = 0.6;
    private static final double SPIN_DECAY_COEFFICIENT = 0.01;
    private static final double MOMENT_OF_INERTIA = 2.0 / 5.0 * 0.21 * 0.075 * 0.075;
    private static final double G_FORCE = 9.81;
    private static final double SIMULATION_TIME_STEP = 0.001;


    /**
     * @param startPose   initial 3-D pose of the ball
     * @param startVel    initial velocity (m/s) in field frame
     * @param initialSpin initial backspin magnitude (rad/s)
     * @param spinAxis    unit vector (field frame) that is the ball's spin axis –
     *                    must be perpendicular to the shot direction in the horizontal plane
     */
    public SimulatedBall(Pose3d startPose, Translation3d startVel, double initialSpin, Translation3d spinAxis) {
        this.pose = startPose;
        this.vel = startVel;
        this.spinRadiansPerSecond = initialSpin;
        this.spinAxis = spinAxis;
    }

    public void update() {
        // Run multiple simulation steps per update for accuracy
        double dt = SIMULATION_TIME_STEP;
        int steps = (int)(0.02 / dt); // 50 Hz → 20 ms per call

        for (int i = 0; i < steps; i++) {
            updateStep(dt);
            lifetime += dt;

            if (pose.getZ() < 0.05 && vel.getZ() < 0) {
                active = false;
                break;
            }

            if (lifetime > 5.0) {
                active = false;
                break;
            }
        }
    }

    private void updateStep(double dt) {
        // 1. Gravity
        Translation3d gravityAcc = new Translation3d(0, 0, -G_FORCE);

        // 2. Aerodynamic drag
        Translation3d dragAcc = calculateDragAcceleration();

        // 3. Magnus effect (backspin lift)
        Translation3d magnusAcc = calculateMagnusAcceleration();

        // 4. Spin decay
        updateSpinDecay(dt);

        // 5. Integrate
        Translation3d totalAcc = gravityAcc.plus(dragAcc).plus(magnusAcc);
        vel  = vel.plus(totalAcc.times(dt));
        pose = pose.plus(new Transform3d(vel.times(dt), new Rotation3d()));
    }

    private Translation3d calculateDragAcceleration() {
        double vMag = vel.getNorm();
        if (vMag < 1e-6) return new Translation3d();

        double dragForceMag = 0.5 * AIR_DENSITY * vMag * vMag * DRAG_COEFFICIENT * GAME_PIECE_AREA;
        double dragAccMag   = dragForceMag / GAME_PIECE_MASS_KG;

        // Drag opposes velocity
        return vel.div(vMag).times(-dragAccMag);
    }

    private Translation3d calculateMagnusAcceleration() {
        double vMag = vel.getNorm();

        if (vMag < 1e-6 || Math.abs(spinRadiansPerSecond) < 1e-6) return new Translation3d();

        // Dimensionless spin parameter
        double spinParameter         = (spinRadiansPerSecond * 0.075) / vMag;
        double magnusLiftCoefficient = MAGNUS_LIFT_FACTOR * spinParameter;
        double magnusForceMag        = 0.5 * AIR_DENSITY * vMag * vMag * magnusLiftCoefficient * GAME_PIECE_AREA;
        double magnusAccMag          = magnusForceMag / GAME_PIECE_MASS_KG;

        // Magnus direction = spinAxis × vel
        double ax = spinAxis.getX(), ay = spinAxis.getY(), az = spinAxis.getZ();
        double bx = vel.getX(),      by = vel.getY(),      bz = vel.getZ();

        Translation3d magnusDir = new Translation3d(
                ay * bz - az * by,
                az * bx - ax * bz,
                ax * by - ay * bx);

        double magnusDirMag = magnusDir.getNorm();
        if (magnusDirMag < 1e-6) return new Translation3d();

        return magnusDir.div(magnusDirMag).times(magnusAccMag);
    }

    private void updateSpinDecay(double dt) {
        double vMag       = vel.getNorm();
        double coefficient = (0.5 * SPIN_DECAY_COEFFICIENT * AIR_DENSITY * GAME_PIECE_AREA) / MOMENT_OF_INERTIA;
        spinRadiansPerSecond -= coefficient * spinRadiansPerSecond * dt * vMag;
    }

    public Pose3d getPose()          { return pose; }
    public boolean isActive()        { return active; }
    public Translation3d getVelocity() { return vel; }
    public double getSpin()          { return spinRadiansPerSecond; }
}
