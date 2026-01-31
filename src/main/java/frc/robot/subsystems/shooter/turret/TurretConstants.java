package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimulationProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimulationProperties.SimulationType.SIMPLE_MOTOR;
import static frc.robot.utilities.PortsConstants.TurretPorts.TURRET_MOTOR_PORT;

public class TurretConstants extends GenericSubsystem {
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(
            //todo: Center of robot to turret. From Sirtut!
    );

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
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;

        configuration.slot = new MotorProperties.Slot(1, 0, 0, 0, 0, 0);//TODO TUNE
        configuration.profileMaxVelocity = 1.069;//TODO TUNE
        configuration.profileMaxAcceleration = 1.57; //TODO TUNE

        configuration.statorCurrentLimit = 40; //TODO TUNE
        configuration.gearRatio = 100.0; //TODO TUNE

        configuration.forwardSoftLimit = MAX_ANGLE.getRotations();
        configuration.reverseSoftLimit = MIN_ANGLE.getRotations();

        configuration.simulationSlot = new MotorProperties.Slot(0, 0, 0, 11.22, 0, 0);
        configuration.simulationProperties = new SimulationProperties.Slot(SIMPLE_MOTOR, getFalcon500(1), 100, 0.045);

        TURRET_MOTOR.configure(configuration);

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION); //TODO verify if needed
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}