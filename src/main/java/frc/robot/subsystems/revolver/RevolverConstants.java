package frc.robot.subsystems.revolver;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.RevolverPorts.REVOLVER_MOTOR_PORT;

public class RevolverConstants {
    protected static final Motor REVOLVER_MOTOR = MotorFactory.createSpark("Revolver Motor", REVOLVER_MOTOR_PORT, FLEX);

    static {
        configureRevolverMotor();
    }

    private static void configureRevolverMotor() {
        final MotorConfiguration revolverMotorConfiguration = new MotorConfiguration();

        revolverMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        revolverMotorConfiguration.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        revolverMotorConfiguration.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.001);

        REVOLVER_MOTOR.configure(revolverMotorConfiguration);

        REVOLVER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
    }
}