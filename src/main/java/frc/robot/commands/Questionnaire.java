package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.flippable.FlippablePose2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.math.geometry.Rotation2d.kZero;
import static frc.robot.RobotContainer.INTAKE;
import static frc.robot.RobotContainer.SHOOTER_STATES;
import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindAndFollow;
import static frc.robot.subsystems.intake.IntakeConstants.IntakeState.DEPLOYED;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.IDLE;
import static frc.robot.subsystems.shooter.ShooterStates.ShooterState.SHOOTING_HUB;
import static frc.robot.subsystems.swerve.SwerveCommands.driveWithTimeout;
import static frc.robot.utilities.FieldConstants.*;

public class Questionnaire {
    private static final Transform2d TRENCH_TO_ROBOT_START = new Transform2d(new Translation2d(-1, 0), kZero);

    private final LoggedDashboardChooser<StartingPose> CHOOSE_STARTING_POSE;
    private final LoggedDashboardChooser<CollectionPose> CHOOSE_ALLIANCE_COLLECTION;

    private enum StartingPose {
        TRENCH_BOTTOM(new FlippablePose2d(BOTTOM_TRENCH.getMiddle(), kZero, false, true),
                BALLS_BOTTOM_START),
        TRENCH_TOP(new FlippablePose2d(BOTTOM_TRENCH.mirroredY().getMiddle(), kZero, false, true),
                BALLS_TOP_START);

        private final FlippablePose2d startingPose;
        private final FlippablePose2d beginIntakingPose;

        StartingPose(FlippablePose2d startingPose, FlippablePose2d beginIntakingPose) {
            this.startingPose = startingPose;
            this.beginIntakingPose = beginIntakingPose;
        }

        public Pose2d getPose() {
            return startingPose.get().transformBy(TRENCH_TO_ROBOT_START);
        }

        public Pose2d getBeginIntakingPose() {
            return beginIntakingPose.get();
        }
    }

    private enum CollectionPose {
        DEPOT(DEPOT_LOCATION),
        OUTPOST(OUTPOST_LOCATION),
        SHOOT_BOTTOM(new FlippablePose2d(new Translation2d(2.604766, HALF_FIELD_WIDTH - 2), kZero, false, true)),
        SHOOT_TOP(new FlippablePose2d(new Translation2d(2.604766, HALF_FIELD_WIDTH + 2), kZero, false, true));

        private final FlippablePose2d startingPose;

        CollectionPose(FlippablePose2d startingPose) {
            this.startingPose = startingPose;
        }

        public Pose2d getPose() {
            return startingPose.get();
        }
    }

    public Questionnaire() {
        CHOOSE_STARTING_POSE = createQuestion("Which trench side?", StartingPose.class);
        CHOOSE_ALLIANCE_COLLECTION = createQuestion("Where to collect from?", CollectionPose.class);
    }

    public Command getCommand() {
        final StartingPose start = CHOOSE_STARTING_POSE.get();
        final CollectionPose collect = CHOOSE_ALLIANCE_COLLECTION.get();

        if (start == null || collect == null) return null;

        final Command intakeAndFollowPath = driveWithTimeout(-0.15, 0, 0, true, 3)
                        .alongWith(INTAKE.setState(DEPLOYED));

        return SHOOTER_STATES.setState(IDLE).alongWith(pathfindAndFollow(start.getPose()))
                .andThen(pathfindAndFollow(start.getBeginIntakingPose()))
                .andThen(intakeAndFollowPath)
                .andThen(pathfindAndFollow(collect.getPose()).alongWith(SHOOTER_STATES.setState(SHOOTING_HUB)));
    }

    public String getSelected() {
        final StartingPose start = CHOOSE_STARTING_POSE.get();
        final CollectionPose collect = CHOOSE_ALLIANCE_COLLECTION.get();

        return (start == null || collect == null) ? "Custom" : start.name() + " + " + collect.name();
    }

    private <T extends Enum<T>> LoggedDashboardChooser<T> createQuestion(String questionName, Class<T> enumClass) {
        final LoggedDashboardChooser<T> question = new LoggedDashboardChooser<>(questionName);

        for (final T option : enumClass.getEnumConstants()) {
            question.addOption(option.name(), option);
        }

        if (enumClass.getEnumConstants().length > 0) {
            question.addDefaultOption(enumClass.getEnumConstants()[0].name(), enumClass.getEnumConstants()[0]);
        }

        return question;
    }
}