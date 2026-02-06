package frc.lib.generic.characterization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.encoder.Encoder;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class GearRatioCharacterization extends Command {
    private static final LoggedNetworkNumber MOVEMENT_VOLTAGE = new LoggedNetworkNumber("GearRatioCharacterization/Voltage", 1.0);
    private static final LoggedNetworkBoolean SHOULD_MOVE_CLOCKWISE = new LoggedNetworkBoolean("GearRatioCharacterization/ShouldMoveClockwise", false);

    private final Motor motor;
    private final Encoder encoder;
    private final MotorProperties.IdleMode startingIdleMode;

    private double startingMotorPosition;
    private double startingEncoderPosition;
    private double gearRatio;

    public GearRatioCharacterization(GenericSubsystem requirement, Motor motor, Encoder encoder) {
        this.motor = motor;
        this.encoder = encoder;

        startingIdleMode = motor.getConfig().idleMode;

        this.addRequirements(requirement);
    }

    @Override
    public void initialize() {
        startingMotorPosition = getMotorDistance();
        startingEncoderPosition = getEncoderDistance();

        motor.setIdleMode(MotorProperties.IdleMode.BRAKE);
    }

    @Override
    public void execute() {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, MOVEMENT_VOLTAGE.get() * getVoltageSign());
        gearRatio = getMotorDistance() / getEncoderDistance();

        Logger.recordOutput("GearRatioCalculation/MotorDistance", getMotorDistance());
        Logger.recordOutput("GearRatioCalculation/EncoderDistance", getEncoderDistance());
        Logger.recordOutput("GearRatioCalculation/GearRatio", gearRatio);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Gear Ratio: " + gearRatio);
        motor.setIdleMode(startingIdleMode);
    }

    private double getMotorDistance() {
        return startingMotorPosition - motor.getMotorPosition();
    }

    private double getEncoderDistance() {
        return startingEncoderPosition - encoder.getEncoderPosition();
    }

    private int getVoltageSign() {
        return SHOULD_MOVE_CLOCKWISE.get()  ? -1 : 1;
    }
}
