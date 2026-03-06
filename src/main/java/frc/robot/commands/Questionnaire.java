package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.flippable.FlippableTranslation2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.commands.pathfinding.PathfindingCommands.pathfindAndFollow;
import static frc.robot.utilities.FieldConstants.BOTTOM_TRENCH;

public class Questionnaire {
    private final LoggedDashboardChooser<String> CHOOSE_STARTING_POSE;
//    private final LoggedDashboardChooser<String> CHOOSE_MID_COLLECTION; //Till middle and back from same trench, till end and back from other trench
//    private final LoggedDashboardChooser<String> CHOOSE_ALLIANCE_COLLECTION; //When in alliance zone, SHoot REGARDLEss. but, should collect from HP && depot

    private enum StartingPose {
        TRENCH_BOTTOM(new FlippableTranslation2d(BOTTOM_TRENCH.getMiddle(), true)),
        TRENCH_TOP(new FlippableTranslation2d(BOTTOM_TRENCH.mirroredY().getMiddle(), true));

        private final FlippableTranslation2d startingPose;

        StartingPose(FlippableTranslation2d startingPose) {
            this.startingPose = startingPose;
        }

        public Translation2d getStartingPose() {
            return startingPose.get();
        }

        public Translation2d fromString(String name) {
            if (name == TRENCH_BOTTOM.name()) return getStartingPose();
            if (name == TRENCH_TOP.name()) return getStartingPose();
            return  getStartingPose();
        }
    }

//    private enum MidCollectionPose {
//        BALLS_MIDDLE(new FlippableTranslation2d(BOTTOM_TRENCH.getMiddle(), true)),
//        BALLS_END(new FlippableTranslation2d(BOTTOM_TRENCH.mirroredY().getMiddle(), true));
//
//        private final FlippableTranslation2d startingPose;
//        private final FlippableTranslation2d collectionEndPose;
//
//        MidCollectionPose(FlippableTranslation2d startingPose) {
//            this.startingPose = startingPose;
//        }
//
//        public Translation2d getCollectionEndPose() {
//            return collectionEndPose.get();
//        }
//
//        public Translation2d getReturnPose() {
//
//        }
//    }

    public Questionnaire() {
        CHOOSE_STARTING_POSE = createQuestion();
    }

    public Command getCommand() {
        return null;
//        return pathfindAndFollow(CHOOSE_STARTING_POSE.get()); //todo wow this is shi
    }

    public String getSelected() {
        return CHOOSE_STARTING_POSE.getSendableChooser().getSelected() != "None" ? CHOOSE_STARTING_POSE.get() : "Custom";
    }

    private LoggedDashboardChooser<String> createQuestion() {
        final LoggedDashboardChooser<String> question = new LoggedDashboardChooser<>("Which Auto?");

        for (final StartingPose auto : StartingPose.values()) {
            question.addOption(auto.name(), auto.name());
        }

        return question;
    }
}