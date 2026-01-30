package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utilities.PortsConstants.TurretPorts.TURRET_MOTOR_PORT;

public class TurretConstants extends GenericSubsystem {
    protected static final SysIdRoutine.Config SYSID_TURRET_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    protected static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("Turret Motor", TURRET_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d TURRET_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Turret Mechanism", 5);

    protected static final Rotation2d
            MAX_ANGLE = Rotation2d.fromDegrees(180),
            MIN_ANGLE = Rotation2d.fromDegrees(-180);

    static {
        configureTurretMotor();
    }

    private static void configureTurretMotor() {
        final MotorConfiguration turretMotorConfiguration = new MotorConfiguration();

        turretMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        turretMotorConfiguration.slot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);
        turretMotorConfiguration.profileMaxVelocity = 1.069;
        turretMotorConfiguration.profileMaxAcceleration = 1.57; //TODO TUNE

        turretMotorConfiguration.statorCurrentLimit = 40;

        turretMotorConfiguration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 11.22, 0, 0);
        turretMotorConfiguration.simulationProperties = new SimulationProperties.Slot(
                SimulationProperties.SimulationType.SIMPLE_MOTOR,
                DCMotor.getFalcon500(1),
                100,
                0.045);

        TURRET_MOTOR.configure(turretMotorConfiguration);

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}