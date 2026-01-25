package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createSpeedMechanism;
import static frc.robot.utilities.PortsConstants.FlywheelPort.FLYWHEEL_MOTOR_PORT;
import static frc.robot.utilities.PortsConstants.FlywheelPort.SECOND_FLYWHEEL_PORT;

public class FlywheelConstants {
    protected static final Motor FLYWHEEL_MOTOR = MotorFactory.createTalonFX("Flywheel Motor", FLYWHEEL_MOTOR_PORT);
    protected static final Motor SECONDARY_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("Secondary flywheel Motor", SECOND_FLYWHEEL_PORT);
    protected static final SpeedMechanism2d FLYWHEEL_MECHANISM = createSpeedMechanism("Flywheel Mechanism");

    static {
        configureFlywheelMotor();
    }

    private static void configureFlywheelMotor() {
        final MotorConfiguration flywheelMotorConfiguration = new MotorConfiguration();
        flywheelMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        flywheelMotorConfiguration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);

        flywheelMotorConfiguration.simulationSlot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                150,
                0.2);

        FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);

        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        SECONDARY_FLYWHEEL_MOTOR.setFollower(FLYWHEEL_MOTOR, true);
    }
}