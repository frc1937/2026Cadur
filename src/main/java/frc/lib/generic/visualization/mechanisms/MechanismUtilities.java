package frc.lib.generic.visualization.mechanisms;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import static frc.lib.generic.visualization.mechanisms.MechanismConstants.*;


public class MechanismUtilities {
    protected static LoggedMechanismRoot2d createDefaultRoot(String name, LoggedMechanism2d mechanism) {
        return mechanism.getRoot(name, DEFAULT_ROOT_X, DEFAULT_ROOT_Y);
    }

    protected static void createElevatorOutline(LoggedMechanism2d mechanism) {
        final LoggedMechanismRoot2d outlineRoot = mechanism.getRoot("outlineRoot", 1, 1);

        final LoggedMechanismLigament2d
                outlineTop = new LoggedMechanismLigament2d("outlineTop", 8, -90, DEFAULT_LINE_WIDTH, GRAY),
                outlineLeft = new LoggedMechanismLigament2d("outlineLeft", 18, 90, DEFAULT_LINE_WIDTH, GRAY),
                outlineRight = new LoggedMechanismLigament2d("outlineRight", 18, -90, DEFAULT_LINE_WIDTH, GRAY);

        outlineTop.append(outlineRight);
        outlineRoot.append(outlineLeft);
        outlineLeft.append(outlineTop);
    }
}