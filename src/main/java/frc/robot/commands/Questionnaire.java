package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.flippable.FlippablePose2d;
import frc.lib.util.flippable.FlippableTranslation2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindAndFollow;
import static frc.robot.utilities.FieldConstants.*;

public class Questionnaire {
    private static final Translation2d ROBOT_DISTANCE_FROM_TRENCH = new Translation2d(1, 0);

    private final LoggedDashboardChooser<StartingPose> CHOOSE_STARTING_POSE;
    private final LoggedDashboardChooser<CollectionPose> CHOOSE_ALLIANCE_COLLECTION;

    private enum StartingPose {
        TRENCH_BOTTOM(new FlippableTranslation2d(BOTTOM_TRENCH.getMiddle().minus(ROBOT_DISTANCE_FROM_TRENCH), false, true), BALLS_BOTTOM_START),
        TRENCH_TOP(new FlippableTranslation2d(BOTTOM_TRENCH.mirroredY().getMiddle().minus(ROBOT_DISTANCE_FROM_TRENCH), false, true), BALLS_TOP_START);

        private final FlippableTranslation2d startingPose;
        private final FlippablePose2d beginIntakingPose;

        StartingPose(FlippableTranslation2d startingPose, FlippablePose2d beginIntakingPose) {
            this.startingPose = startingPose;
            this.beginIntakingPose = beginIntakingPose;
        }

        public Translation2d getPose() {
            return startingPose.get();
        }
        public Pose2d getBeginIntakingPose() {
            return beginIntakingPose.get();
        }
    }

    private enum CollectionPose {
        DEPOT(DEPOT_LOCATION),
        OUTPOST(OUTPOST_LOCATION),
        NONE(new FlippableTranslation2d(2.604766, HALF_FIELD_WIDTH, true));

        private final FlippableTranslation2d startingPose;

        CollectionPose(FlippableTranslation2d startingPose) {
            this.startingPose = startingPose;
        }

        public Translation2d getPose() {
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

        final Command intakeAndFollowPath = pathfindAndFollow(BALLS_MIDDLE.get());

        return pathfindAndFollow(start.getPose())
                .andThen(pathfindAndFollow(start.getBeginIntakingPose()))
                .andThen(intakeAndFollowPath)
                .andThen(pathfindAndFollow(collect.getPose()));
    }

    public String getSelected() {
        final String selected = CHOOSE_STARTING_POSE.getSendableChooser().getSelected();
        return (selected == null || "None".equals(selected)) ? "Custom" : selected;
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