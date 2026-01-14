package frc.lib.generic.visualization.mechanisms;

import static frc.robot.GlobalConstants.SHOULD_DISPLAY_MECHANISMS;

public class MechanismFactory {
    /**
     * if {@code GlobalConstants.SHOULD_DISPLAY_MECHANISMS} is false, do not initiate mechanism to save resources.
     * if it is true, display the mechanism.
     * @see SingleJointedArmMechanism2d
     * @return SingleJointedArmMechanism2d
     */
    public static SingleJointedArmMechanism2d createSingleJointedArmMechanism(String name, double armLength) {
        if (SHOULD_DISPLAY_MECHANISMS)
            return new SingleJointedArmMechanism2d(name, armLength);
        
        return null;
    }

    /**
     * if {@code GlobalConstants.SHOULD_DISPLAY_MECHANISMS} is false, do not initiate mechanism to save resources.
     * if it is true, display the mechanism.
     * @see DoubleJointedArmMechanism2d
     * @return DoubleJointedArmMechanism2d
     */
    public static DoubleJointedArmMechanism2d createDoubleJointedArmMechanism(String name, double shoulderLength, double elbowLength) {
        if (SHOULD_DISPLAY_MECHANISMS)
            return new DoubleJointedArmMechanism2d(name, shoulderLength, elbowLength);

        return null;
    }

    /**
     * if {@code GlobalConstants.SHOULD_DISPLAY_MECHANISMS} is false, do not initiate mechanism to save resources.
     * if it is true, display the mechanism.
     * @see SpeedMechanism2d
     * @return SpeedMechanism2d
     */
    public static SpeedMechanism2d createSpeedMechanism(String name) {
        if (SHOULD_DISPLAY_MECHANISMS)
            return new SpeedMechanism2d(name);

        return null;
    }

    /**
     * if {@code GlobalConstants.SHOULD_DISPLAY_MECHANISMS} is false, do not initiate mechanism to save resources.
     * if it is true, display the mechanism.
     * @see ElevatorMechanism2d
     * @return ElevatorMechanism2d
     */
    public static ElevatorMechanism2d createElevatorMechanism(String name, double elevatorLength) {
        if (SHOULD_DISPLAY_MECHANISMS)
            return new ElevatorMechanism2d(name, elevatorLength);
        
        return null;
    }

    /**
     * if {@code GlobalConstants.SHOULD_DISPLAY_MECHANISMS} is false, do not initiate mechanism to save resources.
     * if it is true, display the mechanism.
     * @see ArmElevatorMechanism2d
     * @return ArmElevatorMechanism2d
     */
    public static ArmElevatorMechanism2d createArmElevatorMechanism(String name, double armLength) {
        if (SHOULD_DISPLAY_MECHANISMS)
            return new ArmElevatorMechanism2d(name, armLength);

        return null;
    }
}