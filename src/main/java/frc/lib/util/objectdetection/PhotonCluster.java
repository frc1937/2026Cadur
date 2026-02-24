package frc.lib.util.objectdetection;

import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCluster {
    private PhotonTrackedTarget closestTarget;
    private double yawSum = 0, pitchSum = 0, areaSum = 0;
    private int count = 0;

    private double[] yaws;
    private double[] pitches;

    private double avgYaw, avgPitch;

    public PhotonCluster(PhotonTrackedTarget initialTarget) {
        this.closestTarget = initialTarget;
        add(initialTarget);
    }

    public void add(PhotonTrackedTarget target) {
        yawSum += target.getYaw();
        pitchSum += target.getPitch();
        areaSum += target.getArea();
        count++;

        avgYaw = yawSum / count;
        avgPitch = pitchSum / count;

        if (target.getArea() > closestTarget.getArea()) {
            closestTarget = target;
        }
    }

    public double calculateDistanceSq(PhotonTrackedTarget target) {
        final double dy = target.getYaw() - avgYaw;
        final double dp = target.getPitch() - avgPitch;
        return (dy * dy) + (dp * dp);
    }

    public boolean isBetterThan(PhotonCluster other) {
        if (this.count != other.count)
            return this.count > other.count;
        return this.closestTarget.getArea() > other.closestTarget.getArea();
    }

    public double getYaw() {return avgYaw;}
    public double[] getYaws() {return yaws.clone();}
    public double getPitch() {return avgPitch;}
    public double[] getPitches() {return pitches.clone();}
    public double getArea() {return areaSum;}
    public int getCount() {return count;}
    public PhotonTrackedTarget getClosestTarget() {return closestTarget;}
}
