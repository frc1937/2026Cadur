package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimulationProperties.SimulationType.SIMPLE_MOTOR;
import static frc.lib.generic.visualization.mechanisms.MechanismFactory.createSpeedMechanism;
import static frc.robot.utilities.PortsConstants.FlywheelPort.LEFT_FLYWHEEL_PORT;
import static frc.robot.utilities.PortsConstants.FlywheelPort.RIGHT_FLYWHEEL_PORT;

public class FlywheelConstants {
    protected static final SysIdRoutine.Config SYSID_FLYWHEEL_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final Motor
            MASTER_LEFT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("Left flywheel Motor", LEFT_FLYWHEEL_PORT),
            SLAVE_RIGHT_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("Right flywheel Motor", RIGHT_FLYWHEEL_PORT);

    protected static final SpeedMechanism2d FLYWHEEL_MECHANISM = createSpeedMechanism("Flywheel Mechanism");

    static {
        configureFlywheelMotors();
    }

    private static void configureFlywheelMotors() {
        final MotorConfiguration flywheelMotorConfiguration = new MotorConfiguration();
        flywheelMotorConfiguration.idleMode = MotorProperties.IdleMode.COAST;

        flywheelMotorConfiguration.slot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0); //TODO TUNE

        flywheelMotorConfiguration.statorCurrentLimit = 50;

        flywheelMotorConfiguration.simulationSlot = new MotorProperties.Slot(10, 0, 0, 0, 0, 0);
        flywheelMotorConfiguration.simulationProperties = new SimulationProperties.Slot(SIMPLE_MOTOR, getFalcon500(1), 150, 0.2);

        MASTER_LEFT_FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);
        SLAVE_RIGHT_FLYWHEEL_MOTOR.configure(flywheelMotorConfiguration);

        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_LEFT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        SLAVE_RIGHT_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET); //TODO: Check if needed

        SLAVE_RIGHT_FLYWHEEL_MOTOR.setFollowerOf(MASTER_LEFT_FLYWHEEL_MOTOR, true);
    }
}