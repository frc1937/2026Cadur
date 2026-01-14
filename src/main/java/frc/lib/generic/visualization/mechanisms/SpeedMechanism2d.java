package frc.lib.generic.visualization.mechanisms;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;
import static frc.lib.generic.visualization.mechanisms.MechanismUtilities.createDefaultRoot;

public class SpeedMechanism2d {
    private final String name;
    private final LoggedMechanism2d speedMechanism;
    private final LoggedMechanismRoot2d root;
    private LoggedMechanismLigament2d
            currentSpeedLigament,
            targetSpeedLigament,
            currentArrowTopLigament,
            currentArrowBottomLigament,
            targetArrowTopLigament,
            targetArrowBottomLigament;

    public SpeedMechanism2d(String name) {
        this.name = "Mechanism/" + name;
        this.speedMechanism = new LoggedMechanism2d(DEFAULT_CANVAS_WIDTH, DEFAULT_CANVAS_HEIGHT);
        this.root = createDefaultRoot("speedMechanismRoot", speedMechanism);

        createCurrentLigament();
        createTargetLigament();
    }

    public void updateCurrentSpeed(double speedRPS) {
        if (speedRPS < SLOWEST_SPEED_TO_CONSIDER_IDLE_RPS) {
            currentSpeedLigament.setColor(RED);
            currentArrowTopLigament.setColor(RED);
            currentArrowBottomLigament.setColor(RED);

            currentArrowTopLigament.setAngle(ARROW_TOP_ANGLE_INVERSE);
            currentArrowBottomLigament.setAngle(ARROW_BOTTOM_ANGLE_INVERSE);
        } else if (speedRPS > SLOWEST_SPEED_TO_CONSIDER_IDLE_RPS) {
            currentSpeedLigament.setColor(GREEN);
            currentArrowTopLigament.setColor(GREEN);
            currentArrowBottomLigament.setColor(GREEN);

            currentArrowTopLigament.setAngle(ARROW_TOP_ANGLE);
            currentArrowBottomLigament.setAngle(ARROW_BOTTOM_ANGLE);
        } else {
            currentSpeedLigament.setColor(GRAY);
            currentArrowTopLigament.setColor(GRAY);
            currentArrowBottomLigament.setColor(GRAY);

            currentArrowTopLigament.setAngle(180);
            currentArrowBottomLigament.setAngle(-180);
        }

        currentSpeedLigament.setLength(speedRPS / RPS_TO_LENGTH_FACTOR);
        Logger.recordOutput(name, speedMechanism);
    }

    public void updateTargetSpeed(double targetSpeedRPS) {
        if (targetSpeedRPS < SLOWEST_SPEED_TO_CONSIDER_IDLE_RPS) {
            targetArrowTopLigament.setAngle(ARROW_TOP_ANGLE_INVERSE);
            targetArrowBottomLigament.setAngle(ARROW_BOTTOM_ANGLE_INVERSE);
        } else if (targetSpeedRPS > SLOWEST_SPEED_TO_CONSIDER_IDLE_RPS) {
            targetArrowTopLigament.setAngle(ARROW_TOP_ANGLE);
            targetArrowBottomLigament.setAngle(ARROW_BOTTOM_ANGLE);
        } else {
            targetArrowTopLigament.setAngle(180);
            targetArrowBottomLigament.setAngle(-180);
        }

        targetSpeedLigament.setLength(targetSpeedRPS / RPS_TO_LENGTH_FACTOR);
        Logger.recordOutput(name, speedMechanism);
    }

    private void createCurrentLigament() {
        currentSpeedLigament = new LoggedMechanismLigament2d("currentSpeed", 5, 0, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowTopLigament = new LoggedMechanismLigament2d("currentArrowTop", 1, ARROW_TOP_ANGLE, DEFAULT_LINE_WIDTH, GREEN);
        currentArrowBottomLigament = new LoggedMechanismLigament2d("currentArrowBottom", 1, ARROW_BOTTOM_ANGLE, DEFAULT_LINE_WIDTH, GREEN);

        currentSpeedLigament.append(currentArrowTopLigament);
        currentSpeedLigament.append(currentArrowBottomLigament);
        root.append(currentSpeedLigament);
    }

    private void createTargetLigament() {
        targetSpeedLigament = new LoggedMechanismLigament2d("targetSpeed", 5, 0, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowTopLigament = new LoggedMechanismLigament2d("targetArrowTop", 1, ARROW_TOP_ANGLE, DEFAULT_LINE_WIDTH, GRAY);
        targetArrowBottomLigament = new LoggedMechanismLigament2d("targetArrowBottom", 1, ARROW_BOTTOM_ANGLE, DEFAULT_LINE_WIDTH, GRAY);

        targetSpeedLigament.append(targetArrowTopLigament);
        targetSpeedLigament.append(targetArrowBottomLigament);
        root.append(targetSpeedLigament);
    }
}