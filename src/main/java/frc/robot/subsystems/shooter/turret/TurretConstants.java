package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

import static edu.wpi.first.math.system.plant.DCMotor.getFalcon500;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.generic.simulation.SimProperties.SimulationType.SIMPLE_MOTOR;
import static frc.robot.utilities.PortsConstants.TurretPorts.TURRET_MOTOR_PORT;

public class TurretConstants extends GenericSubsystem {
    public static final Transform3d ROBOT_TO_CENTER_TURRET = new Transform3d(
            new Translation3d(0.2, 0, 0), new Rotation3d(0, 0,0) //TODO: add turret height FROM THE FLOOR
            //todo: Center of robot to turret. From Sirtut!
    );

    public static final Transform3d TURRET_CENTER_TO_CAMERA = new Transform3d(
            new Translation3d(0.2, 0, 0), new Rotation3d(0, 0,0)
            //todo: Center turret to CAMERA. From Sirtut!
    );

    public static final double TURRET_ANGLE_TOLERANCE_ROTATIONS = 2.0 / 360.0;

    protected static final SysIdRoutine.Config SYSID_TURRET_CONFIG = new SysIdRoutine.Config(
            Volts.per(Second).of(1),
            Volts.of(2),
            Second.of(5)
    );

    public static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("TURRET_MOTOR", TURRET_MOTOR_PORT);
    protected static final SingleJointedArmMechanism2d TURRET_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Turret Mechanism", 5);

    protected static final Rotation2d
            MAX_ANGLE = Rotation2d.fromDegrees(270),
            MIN_ANGLE = Rotation2d.fromDegrees(-270);

    static {
        configureTurretMotor();
    }

    private static void configureTurretMotor() {
        final MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;
        configuration.inverted = true;

        // kP and kD provide position error correction; without them pure-feedforward drifts unchecked.
        // kV/kA/kS values are from SysId characterisation.
        configuration.slot = new MotorProperties.Slot(2.0, 0, 0.1, 2.9229, 0.10831, 0.22603);

        configuration.profileMaxVelocity = 2;
        configuration.profileMaxAcceleration = 3.0;

        configuration.statorCurrentLimit = 40;
        configuration.gearRatio = 23.8327;
        configuration.closedLoopTolerance = 1.0 / 360; //todo tune

        configuration.forwardSoftLimit = MAX_ANGLE.getRotations();
        configuration.reverseSoftLimit = MIN_ANGLE.getRotations();

        configuration.simulationSlot = new MotorProperties.Slot(5, 0, 0.005, 11.22, 0, 0);
        configuration.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, getFalcon500(1), 100, 0.045);

        TURRET_MOTOR.configure(configuration);

        TURRET_MOTOR.setMotorEncoderPosition(0);

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
    }
}