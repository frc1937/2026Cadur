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

    // Physics constants
    private static final double AIR_DENSITY = 1.205;
    private static final double DRAG_COEFFICIENT = 0.5;
    private static final double GAME_PIECE_MASS_KG = 0.21;
    private static final double GAME_PIECE_AREA = Math.PI * 0.075 * 0.075;
    private static final double MAGNUS_LIFT_FACTOR = 0.6;
    private static final double SPIN_DECAY_COEFFICIENT = 0.01;
    private static final double MOMENT_OF_INERTIA = 2.0 / 5.0 * 0.21 * 0.075 * 0.075;
    private static final Translation3d MAGNUS_SPIN_AXIS = new Translation3d(0.0, 1.0, 0.0);
    private static final double G_FORCE = 9.81;
    private static final double SIMULATION_TIME_STEP = 0.001;


    public SimulatedBall(Pose3d startPose, Translation3d startVel, double initialSpin) {
        this.pose = startPose;
        this.vel = startVel;
        this.spinRadiansPerSecond = initialSpin;
    }

    public void update() {
        // Run multiple simulation steps per update for accuracy
        double dt = SIMULATION_TIME_STEP;
        int steps = (int)(0.02 / dt); // Assuming 50Hz update rate (0.02s)

        for (int i = 0; i < steps; i++) {
            updateStep(dt);
            lifetime += dt;

            // Check termination conditions
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
        // 1. Calculate gravity
        Translation3d gravityAcc = new Translation3d(0, 0, -G_FORCE);

        // 2. Calculate drag
        Translation3d dragAcc = calculateDragAcceleration();

        // 3. Calculate Magnus effect
        Translation3d magnusAcc = calculateMagnusAcceleration();

        // 4. Update spin decay
        updateSpinDecay(dt);

        // 5. Combine all accelerations
        Translation3d totalAcc = gravityAcc.plus(dragAcc).plus(magnusAcc);

        // 6. Update velocity: v = v + a * dt
        vel = vel.plus(totalAcc.times(dt));

        // 7. Update position: p = p + v * dt
        pose = pose.plus(new Transform3d(vel.times(dt), new Rotation3d()));
    }

    private Translation3d calculateDragAcceleration() {
        double vMag = vel.getNorm();
        if (vMag < 1e-6) {
            return new Translation3d();
        }

        double dragForceMag = 0.5 * AIR_DENSITY * Math.pow(vMag, 2) * DRAG_COEFFICIENT * GAME_PIECE_AREA;
        double dragAccMag = dragForceMag / GAME_PIECE_MASS_KG;

        // Drag is opposite to velocity direction
        Translation3d velocityDir = vel.div(vMag);
        return velocityDir.times(-dragAccMag);
    }

    private Translation3d calculateMagnusAcceleration() {
        double vMag = vel.getNorm();

        // Guard clause for near-zero velocity or spin
        if (vMag < 1e-6 || Math.abs(spinRadiansPerSecond) < 1e-6) {
            return new Translation3d();
        }

        // 1. Calculate Magnus acceleration magnitude
        double spinParameter = (spinRadiansPerSecond * 0.075) / vMag;
        double magnusLiftCoefficient = MAGNUS_LIFT_FACTOR * spinParameter;
        double magnusForceMag = 0.5 * 1.225 * vMag * vMag * magnusLiftCoefficient * GAME_PIECE_AREA;
        double magnusAccMag = magnusForceMag / GAME_PIECE_MASS_KG;

        // 2. Manual Cross Product: MAGNUS_SPIN_AXIS x vel
        // Axis (a) components
        double ax = MAGNUS_SPIN_AXIS.getX();
        double ay = MAGNUS_SPIN_AXIS.getY();
        double az = MAGNUS_SPIN_AXIS.getZ();

        // Velocity (b) components
        double bx = vel.getX();
        double by = vel.getY();
        double bz = vel.getZ();

        // Cross product formula
        double resX = ay * bz - az * by;
        double resY = az * bx - ax * bz;
        double resZ = ax * by - ay * bx;

        Translation3d magnusDir = new Translation3d(resX, resY, resZ);
        double magnusDirMag = magnusDir.getNorm();

        if (magnusDirMag < 1e-6) {
            return new Translation3d();
        }

        // 3. Normalize and scale
        return magnusDir.div(magnusDirMag).times(magnusAccMag);
    }

    private void updateSpinDecay(double dt) {
        double vMag = vel.getNorm();
        double coefficient = (0.5 * SPIN_DECAY_COEFFICIENT * AIR_DENSITY * GAME_PIECE_AREA) / MOMENT_OF_INERTIA;
        spinRadiansPerSecond -= coefficient * spinRadiansPerSecond * dt * vMag;
    }

    public Pose3d getPose() { return pose; }
    public boolean isActive() { return active; }

    // For debugging
    public Translation3d getVelocity() { return vel; }
    public double getSpin() { return spinRadiansPerSecond; }
}