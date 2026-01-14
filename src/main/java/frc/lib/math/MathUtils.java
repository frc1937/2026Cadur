package frc.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class MathUtils {
    public static double round(double value, int places) {
        final double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    public static Rotation2d getAngleFromPoseToPose(Pose2d firstPose, Pose2d secondPose) {
        final Translation2d differenceInXY = secondPose.getTranslation().minus(firstPose.getTranslation());

        return Rotation2d.fromRadians(Math.atan2(
                differenceInXY.getY(),
                differenceInXY.getX()));
    }

    public static Rotation2d getAngleFromPoseToPose(Translation2d firstPose, Translation2d secondPose) {
        final Translation2d differenceInXY = secondPose.minus(firstPose);

        return Rotation2d.fromRadians(Math.atan2(
                differenceInXY.getY(),
                differenceInXY.getX()));
    }

    public static Rotation2d getPitchFromPoseToPose(Pose3d firstPose, Pose3d secondPose) {
        return Rotation2d.fromRadians(
                Math.atan2(
                secondPose.getTranslation().getZ() - firstPose.getTranslation().getZ(),
                secondPose.getTranslation().getDistance(firstPose.getTranslation())));
    }
}
