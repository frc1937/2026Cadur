package frc.robot.poseestimation.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.QuestNav;

public class QuestReal extends QuestIO {
    private final QuestNav questNav = new QuestNav();
    private final Transform2d robotToQuest;

    public QuestReal(Transform2d robotToQuest) {
        this.robotToQuest = robotToQuest;
    }

    @Override
    public void setQuestFieldPose(Pose2d robotPose) {
        questNav.setPose(new Pose3d(robotPose.transformBy(robotToQuest)));
    }

    @Override
    public void updateInputs(QuestIOInputsAutoLogged inputs) {
        inputs.connected = questNav.isConnected();
        inputs.tracking = questNav.isTracking();

        if (inputs.connected) {
            inputs.batteryPercent = questNav.getBatteryPercent().getAsInt();
            inputs.timestamp = questNav.getAppTimestamp().getAsDouble();
        }else{
            inputs.batteryPercent = 0;
            inputs.timestamp = 0;
        }

        inputs.robotPose = questNav.getAllUnreadPoseFrames()[0].questPose3d().toPose2d().transformBy(robotToQuest.inverse());

        questNav.commandPeriodic();
    }
}
