package frc.robot.subsystems.revolver;


import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;

import static frc.lib.generic.hardware.motor.MotorProperties.SparkType.FLEX;
import static frc.robot.utilities.PortsConstants.RevolverPorts.REVOLVER_MOTOR_PORT;

public class RevolverConstants {
    protected static final Motor REVOLVER_MOTOR = MotorFactory.createSpark("REVOLVER_MOTOR", REVOLVER_MOTOR_PORT, FLEX);

    static {
        configureRevolverMotor();
    }

    private static void configureRevolverMotor() {
        final MotorConfiguration config = new MotorConfiguration();

        config.inverted = true;
        config.idleMode = MotorProperties.IdleMode.COAST;
        config.supplyCurrentLimit = 60;

        config.simulationSlot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        config.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getNeoVortex(1),
                1,
                0.001);

        REVOLVER_MOTOR.configure(config);

        REVOLVER_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
    }
}