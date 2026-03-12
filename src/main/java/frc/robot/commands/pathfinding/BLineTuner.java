package frc.robot.commands.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.hardware.controllers.Controller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Set;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;

public class BLineTuner {
    private static final String KEY = "BLineTuner/";

    private final PIDController translationPID;
    private final PIDController rotationPID;
    private final PIDController crossTrackPID;

    private final LoggedNetworkNumber translationP, translationI, translationD;
    private final LoggedNetworkNumber rotP, rotI, rotD;
    private final LoggedNetworkNumber ctP, ctI, ctD;

    private final LoggedNetworkNumber targetX;
    private final LoggedNetworkNumber targetY;
    private final LoggedNetworkNumber targetRotDeg;

    public BLineTuner(PIDController translationPID, PIDController rotationPID, PIDController crossTrackPID) {
        this.translationPID = translationPID;
        this.rotationPID = rotationPID;
        this.crossTrackPID = crossTrackPID;

        translationP = new LoggedNetworkNumber(KEY + "Translation/kP", translationPID.getP());
        translationI = new LoggedNetworkNumber(KEY + "Translation/kI", translationPID.getI());
        translationD = new LoggedNetworkNumber(KEY + "Translation/kD", translationPID.getD());

        rotP = new LoggedNetworkNumber(KEY + "Rotation/kP", rotationPID.getP());
        rotI = new LoggedNetworkNumber(KEY + "Rotation/kI", rotationPID.getI());
        rotD = new LoggedNetworkNumber(KEY + "Rotation/kD", rotationPID.getD());

        ctP = new LoggedNetworkNumber(KEY + "CrossTrack/kP", crossTrackPID.getP());
        ctI = new LoggedNetworkNumber(KEY + "CrossTrack/kI", crossTrackPID.getI());
        ctD = new LoggedNetworkNumber(KEY + "CrossTrack/kD", crossTrackPID.getD());

        targetX = new LoggedNetworkNumber(KEY + "Target/+X_meters", 2.0);
        targetY = new LoggedNetworkNumber(KEY + "Target/+Y_meters", 4.0);
        targetRotDeg = new LoggedNetworkNumber(KEY + "Target/Rotation_deg", 0.0);
    }

    public void configureController(Controller controller) {
        controller.getButton(Controller.Inputs.BACK).onTrue(runOnce(this::applyPIDs));
        controller.getButton(Controller.Inputs.LEFT_BUMPER).onTrue(
                Commands.defer(() -> debugPathfindTo(
                           new Pose2d(
                                POSE_ESTIMATOR.getPose().getX() + targetX.get(),
                                POSE_ESTIMATOR.getPose().getY() + targetY.get(),
                                Rotation2d.fromDegrees(targetRotDeg.get()).plus(POSE_ESTIMATOR.getCurrentAngle()))),
                        Set.of(SWERVE))
        );
    }

    /**
     * Push the current Shuffleboard PID values into the live PID controllers and print a summary.
     * Also called by the START button.
     */
    public void applyPIDs() {
        translationPID.setPID(translationP.get(), translationI.get(), translationD.get());
        rotationPID.setPID(rotP.get(), rotI.get(), rotD.get());
        crossTrackPID.setPID(ctP.get(), ctI.get(), ctD.get());

        System.out.println("\n[BLineTuner] === PIDs Applied ===");
        System.out.printf("  Translation  kP=%.4f  kI=%.4f  kD=%.4f%n", translationP.get(), translationI.get(), translationD.get());
        System.out.printf("  Rotation     kP=%.4f  kI=%.4f  kD=%.4f%n", rotP.get(), rotI.get(), rotD.get());
        System.out.printf("  CrossTrack   kP=%.4f  kI=%.4f  kD=%.4f%n", ctP.get(), ctI.get(), ctD.get());
    }

    /**
     * Build a debug path-follow command to {@code target}.
     * Prints start info, streams live errors to console + AdvantageScope, and prints a final summary.
     */
    public Command debugPathfindTo(Pose2d target) {
        return Commands.sequence(
                runOnce(() -> printStartInfo(target)),
                Commands.race(
                        PathfindingCommands.pathfindAndFollow(target),
                        Commands.run(() -> logLiveErrors(target))
                ),
                runOnce(() -> logFinalErrors(target))
        );
    }

    private void printStartInfo(Pose2d target) {
        Pose2d current = POSE_ESTIMATOR.getPose();
        double dist = current.getTranslation().getDistance(target.getTranslation());

        System.out.printf("%n[BLineTuner] ▶ Pathfinding to   X=%.3f  Y=%.3f  Rot=%.1f°%n",
                target.getX(), target.getY(), target.getRotation().getDegrees());
        System.out.printf("[BLineTuner]   From             X=%.3f  Y=%.3f  Rot=%.1f°  (%.2f m away)%n",
                current.getX(), current.getY(), POSE_ESTIMATOR.getCurrentAngle().getDegrees(), dist);
        System.out.printf("[BLineTuner]   Active PIDs:     Tran kP=%.3f  Rot kP=%.3f  CT kP=%.3f%n",
                translationPID.getP(), rotationPID.getP(), crossTrackPID.getP());
    }

    private void logLiveErrors(Pose2d target) {
        Pose2d current = POSE_ESTIMATOR.getPose();
        final double xErr = target.getX() - current.getX();
        final double yErr = target.getY() - current.getY();
        final double rotErr = target.getRotation().minus(POSE_ESTIMATOR.getCurrentAngle()).getDegrees();

        Logger.recordOutput(KEY + "LiveError/X_m", xErr);
        Logger.recordOutput(KEY + "LiveError/Y_m", yErr);
        Logger.recordOutput(KEY + "LiveError/Rot_deg", rotErr);
        Logger.recordOutput(KEY + "LiveError/TotalXY_m", Math.hypot(xErr, yErr));
        Logger.recordOutput(KEY + "LiveError/TargetPose", target);
        Logger.recordOutput(KEY + "LiveError/CurrentPose", current);
    }

    private void logFinalErrors(Pose2d target) {
        Pose2d current = POSE_ESTIMATOR.getPose();
        final double xErr = target.getX() - current.getX();
        final double yErr = target.getY() - current.getY();
        final double rotErr = target.getRotation().minus(POSE_ESTIMATOR.getCurrentAngle()).getDegrees();
        final double totalXY = Math.hypot(xErr, yErr);

        String rating = totalXY < 0.05 ? "GREAT (<5cm)"
                : totalXY < 0.10 ? "OK    (<10cm)"
                : "NEEDS TUNING";

        System.out.println("[BLineTuner] ■ Path ended. Final errors:");
        System.out.printf("    X error:    %+.4f m%n", xErr);
        System.out.printf("    Y error:    %+.4f m%n", yErr);
        System.out.printf("    Rot error:  %+.2f°%n", rotErr);
        System.out.printf("    Total XY:   %.4f m  [%s]%n%n", totalXY, rating);

        Logger.recordOutput(KEY + "FinalError/X_m", xErr);
        Logger.recordOutput(KEY + "FinalError/Y_m", yErr);
        Logger.recordOutput(KEY + "FinalError/Rot_deg", rotErr);
        Logger.recordOutput(KEY + "FinalError/TotalXY_m", totalXY);
    }
}