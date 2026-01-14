package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static frc.robot.utilities.PortsConstants.ArmPorts.ARM_MOTOR_PORT;

public class ArmConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createTalonFX("Arm Motor", ARM_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Arm Mechanism",0.5);


    private static final Rotation2d
            ARM_MINIMUM_ROTATION = Rotation2d.fromDegrees(0),
            ARM_MAXIMUM_ROTATION = Rotation2d.fromDegrees(180);

    static {
        configureArmMotorConfiguration();
    }

    private static void configureArmMotorConfiguration() {
        final MotorConfiguration armMotorConfiguration = new MotorConfiguration();

        armMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        armMotorConfiguration.slot = new MotorProperties.Slot(0.145, 0, 0, 0.084973, 0, 0.13081, 0, Feedforward.Type.ARM);

        armMotorConfiguration.simulationSlot = new MotorProperties.Slot(40,0,1, 0, 0, 0);
        armMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                ARM_MINIMUM_ROTATION,
                ARM_MAXIMUM_ROTATION,
                true);

        ARM_MOTOR.configure(armMotorConfiguration);

        ARM_MOTOR.setMotorEncoderPosition(0);

        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

    }
}