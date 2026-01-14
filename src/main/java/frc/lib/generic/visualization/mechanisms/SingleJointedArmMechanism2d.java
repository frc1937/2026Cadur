package frc.lib.generic.visualization.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createDefaultRoot;

public class SingleJointedArmMechanism2d {
    private final String name;
    private final LoggedMechanism2d armMechanism;
    private final LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d
            currentAngleLigament,
            targetAngleLigament;

    public SingleJointedArmMechanism2d(String name, double armLength) {
        this.name = "Mechanism/" + name;
        this.armMechanism = new LoggedMechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = createDefaultRoot("armRoot", armMechanism);

        createCurrentLigament(armLength);
        createTargetLigament(armLength);
    }

    public void updateCurrentAngle(Rotation2d currentAngle) {
        currentAngleLigament.setAngle(currentAngle);
        Logger.recordOutput(name, armMechanism);
    }

    public void updateTargetAngle(Rotation2d targetAngle) {
        targetAngleLigament.setAngle(targetAngle);
        Logger.recordOutput(name, armMechanism);
    }

    private void createCurrentLigament(double armLength) {
        currentAngleLigament = new LoggedMechanismLigament2d("armLigament", armLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, BLUE);
        root.append(currentAngleLigament);
    }

    private void createTargetLigament(double armLength) {
        targetAngleLigament = new LoggedMechanismLigament2d("targetArmLigament", armLength, DEFAULT_ARM_ANGLE, DEFAULT_LINE_WIDTH, GRAY);
        root.append(targetAngleLigament);
    }
}