package frc.lib.util.objectdetection;

import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class ClusterHandler {
    private static final double CLUSTERING_THRESHOLD_DEG = 4.0; // adjust based on FOV
    private static final double THRESHOLD_SQ = CLUSTERING_THRESHOLD_DEG * CLUSTERING_THRESHOLD_DEG;

    public static PhotonCluster getBestCluster(List<PhotonTrackedTarget> rawTargets) {
        if (rawTargets == null || rawTargets.isEmpty()) return null;

        final List<PhotonCluster> clusters = cluster(rawTargets);
        PhotonCluster best = null;

        for (PhotonCluster cluster : clusters) {
            if (best == null || cluster.isBetterThan(best)) {
                best = cluster;
            }
        }

        return best;
    }

    private static List<PhotonCluster> cluster(List<PhotonTrackedTarget> rawTargets) {
        final List<PhotonCluster> clusters = new ArrayList<>();

        for (PhotonTrackedTarget target : rawTargets) {
            PhotonCluster closestCluster = null;
            double minDistanceSq = Double.MAX_VALUE;

            for (PhotonCluster cluster : clusters) {
                final double distSq = cluster.calculateDistanceSq(target);
                if (distSq < THRESHOLD_SQ && distSq < minDistanceSq) {
                    minDistanceSq = distSq;
                    closestCluster = cluster;
                }
            }

            if (closestCluster != null)
                closestCluster.add(target);
            else
                clusters.add(new PhotonCluster(target));
        }

        return clusters;
    }
}