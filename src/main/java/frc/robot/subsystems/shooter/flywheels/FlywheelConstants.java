package frc.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.SpeedMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimProperties.SimulationType.SIMPLE_MOTOR;
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
            MASTER_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("MASTER_LEFT_FLYWHEEL_MOTOR", LEFT_FLYWHEEL_PORT),
            SLAVE_FLYWHEEL_MOTOR = MotorFactory.createTalonFX("SLAVE_RIGHT_FLYWHEEL_MOTOR", RIGHT_FLYWHEEL_PORT);

    protected static final SpeedMechanism2d FLYWHEEL_MECHANISM = createSpeedMechanism("Flywheel Mechanism");

    protected static final double FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS = 2;

    static {
        configureFlywheelMotors();
    }

    private static void configureFlywheelMotors() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.slot = new MotorProperties.Slot(99999, 0, 0, 0, 0, 0);

        configuration.bangBangDuty = true;
        configuration.bangBangCurrent = true;

        //max speed: 94 rps kV = 0.1276
        //max acceleration: 44
        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.statorCurrentLimit = 55;
        configuration.closedLoopTolerance = FLYWHEEL_SHOOTING_SPEED_TOLERANCE_RPS;

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 0.1132075472, 0, 0);
        configuration.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getFalcon500(2), 1, 0.002);

        MASTER_FLYWHEEL_MOTOR.configure(configuration);
        SLAVE_FLYWHEEL_MOTOR.configure(configuration);

        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
        MASTER_FLYWHEEL_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);

        SLAVE_FLYWHEEL_MOTOR.setFollowerOf(MASTER_FLYWHEEL_MOTOR, true);
    }
}