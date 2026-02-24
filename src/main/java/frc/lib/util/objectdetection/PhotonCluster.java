package frc.lib.util.objectdetection;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonCluster {
    private PhotonTrackedTarget closestTarget;
    private double yawSum = 0, pitchSum = 0, areaSum = 0;
    private int count = 0;

    private final List<Double> yaws = new ArrayList<>();
    private final List<Double> pitches = new ArrayList<>();

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
        yaws.add(target.getYaw());
        pitches.add(target.getPitch());

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
    public List<Double> getYaws() {return List.copyOf(yaws);}
    public double getPitch() {return avgPitch;}
    public List<Double> getPitches() {return List.copyOf(pitches);}
    public double getArea() {return areaSum;}
    public int getCount() {return count;}
    public PhotonTrackedTarget getClosestTarget() {return closestTarget;}
}
