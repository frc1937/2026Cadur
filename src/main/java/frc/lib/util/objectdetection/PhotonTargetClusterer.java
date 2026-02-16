package frc.lib.util.objectdetection;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class PhotonTargetClusterer {
    private static final double CLUSTERING_THRESHOLD_DEG = 4.0; // adjust based on FOV
    private static final double THRESHOLD_SQ = CLUSTERING_THRESHOLD_DEG * CLUSTERING_THRESHOLD_DEG;

    public static PhotonTargetCluster getBestCluster(List<PhotonTrackedTarget> rawTargets) {
        if (rawTargets == null || rawTargets.isEmpty()) return null;

        final List<PhotonTargetCluster> clusters = cluster(rawTargets);
        PhotonTargetCluster best = null;

        for (PhotonTargetCluster cluster : clusters) {
            if (best == null || cluster.isBetterThan(best)) {
                best = cluster;
            }
        }

        return best;
    }

    private static List<PhotonTargetCluster> cluster(List<PhotonTrackedTarget> rawTargets) {
        final List<PhotonTargetCluster> clusters = new ArrayList<>();

        for (PhotonTrackedTarget target : rawTargets) {
            PhotonTargetCluster closestCluster = null;
            double minDistanceSq = Double.MAX_VALUE;

            for (PhotonTargetCluster cluster : clusters) {
                final double distSq = cluster.calculateDistanceSq(target);
                if (distSq < THRESHOLD_SQ && distSq < minDistanceSq) {
                    minDistanceSq = distSq;
                    closestCluster = cluster;
                }
            }

            if (closestCluster != null)
                closestCluster.add(target);
            else
                clusters.add(new PhotonTargetCluster(target));
        }

        return clusters;
    }

    public static class PhotonTargetCluster {
        private PhotonTrackedTarget closestTarget;
        private double yawSum = 0, pitchSum = 0, areaSum = 0;
        private int count = 0;

        private double avgYaw, avgPitch;

        public PhotonTargetCluster(PhotonTrackedTarget initialTarget) {
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

        public boolean isBetterThan(PhotonTargetCluster other) {
            if (this.count != other.count)
                return this.count > other.count;
            return this.closestTarget.getArea() > other.closestTarget.getArea();
        }

        public double getYaw() {return avgYaw;}
        public double getPitch() {return avgPitch;}
        public double getArea() {return areaSum;}
        public int getCount() {return count;}
        public PhotonTrackedTarget getRepresentativeTarget() {return closestTarget;}
    }
}