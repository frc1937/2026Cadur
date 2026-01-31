package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimulationProperties.SimulationType.ARM;
import static frc.robot.utilities.PortsConstants.HoodPorts.HOOD_MOTOR_PORT;

public class HoodConstants {
    protected static final SysIdRoutine.Config SYSID_HOOD_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final Motor HOOD_MOTOR = MotorFactory.createTalonFX("Hood Motor", HOOD_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d HOOD_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Hood Mechanism", 0.5);

    protected static final Rotation2d
            MIN_ANGLE = Rotation2d.fromDegrees(30),
            MAX_ANGLE = Rotation2d.fromDegrees(85);

    static {
        configureHoodMotorConfiguration();
    }

    private static void configureHoodMotorConfiguration() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;

        configuration.slot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0, 0, Feedforward.Type.ARM); // TODO TUNE - kP=1 placeholder
        configuration.profileMaxVelocity = 1.069;//TODO TUNE
        configuration.profileMaxAcceleration = 1.57; //TODO TUNE

        configuration.statorCurrentLimit = 40; //TODO TUNE
        configuration.gearRatio = 100.0; //TODO TUNE

        configuration.forwardSoftLimit = MAX_ANGLE.getRotations();
        configuration.reverseSoftLimit = MIN_ANGLE.getRotations();

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 11.2240, 0, 0);
        configuration.simulationProperties = new SimulationProperties.Slot(
                ARM,
                getFalcon500(1),
                100,
                0.20,
                0.1,
                MIN_ANGLE,
                MAX_ANGLE,
                false);

        HOOD_MOTOR.configure(configuration);

        HOOD_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
        HOOD_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}