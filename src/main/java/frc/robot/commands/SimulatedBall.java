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

    public SimulatedBall(Pose3d startPose, Translation3d startVel) {
        this.pose = startPose;
        this.vel = startVel;
    }

    public void update(double dt, double airDensity, double dragCoeff, double area, double mass) {
        double vMag = vel.getNorm();
        double dragForceMag = 0.5 * airDensity * Math.pow(vMag, 2) * dragCoeff * area;

        lifetime += dt;

        // Calculate drag acceleration (opposite direction of velocity)
        Translation3d dragAcc = (vMag > 1e-6) ?
                vel.times(-dragForceMag / (mass * vMag)) :
                new Translation3d();

        // Combine gravity and drag accelerations
        Translation3d totalAcc = new Translation3d(
                dragAcc.getX(),
                dragAcc.getY(),
                dragAcc.getZ() - 9.81  // Gravity acceleration
        );

        // Update velocity: v = v + a * dt
        vel = vel.plus(totalAcc.times(dt));

        // Update position: p = p + v * dt
        pose = pose.plus(new Transform3d(vel.times(dt), new Rotation3d()));

        if (pose.getZ() < 0 || lifetime > 5.0) active = false;
    }

    public Pose3d getPose() { return pose; }
    public boolean isActive() { return active; }
}