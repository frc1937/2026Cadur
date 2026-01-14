package frc.lib.generic.visualization.mechanisms;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createDefaultRoot;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createElevatorOutline;

public class ElevatorMechanism2d {
    private final String name;
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismRoot2d
            root,
            targetRoot;

    public ElevatorMechanism2d(String name, double elevatorLength) {
        this.name = "Mechanism/" + name;
        this.elevatorMechanism = new LoggedMechanism2d(DEFAULT_CANVAS_WIDTH, 20);

        this.root = createDefaultRoot("elevatorRoot", elevatorMechanism);
        this.targetRoot = createDefaultRoot("elevatorTargetRoot", elevatorMechanism);

        createCurrentLigament(elevatorLength);
        createTargetLigament(elevatorLength);
        createElevatorOutline(elevatorMechanism);
    }

    public void updateCurrentPosition(double position) {
        root.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + position);
        Logger.recordOutput(name, elevatorMechanism);
    }

    public void updateTargetPosition(double targetPosition) {
        targetRoot.setPosition(DEFAULT_ROOT_X, DEFAULT_ROOT_Y + targetPosition);
        Logger.recordOutput(name, elevatorMechanism);
    }

    private void createCurrentLigament(double elevatorLength) {
        final LoggedMechanismLigament2d currentRightLigament = new LoggedMechanismLigament2d("elevatorRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, RED);
        final LoggedMechanismLigament2d currentLeftLigament = new LoggedMechanismLigament2d("elevatorLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, RED);

        root.append(currentRightLigament);
        root.append(currentLeftLigament);
    }

    private void createTargetLigament(double elevatorLength) {
        final LoggedMechanismLigament2d targetRightLigament = new LoggedMechanismLigament2d("targetRightLigament", elevatorLength, 0, DEFAULT_LINE_WIDTH, BLUE);
        final LoggedMechanismLigament2d targetLeftLigament = new LoggedMechanismLigament2d("targetLeftLigament", elevatorLength, 180, DEFAULT_LINE_WIDTH, BLUE);

        targetRoot.append(targetRightLigament);
        targetRoot.append(targetLeftLigament);
    }
}