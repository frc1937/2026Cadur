package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorInputs;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.lib.math.Optimizations;

import static edu.wpi.first.math.geometry.Rotation2d.fromRotations;
import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_DIAMETER;

public class SwerveModule {
    private static final double PRECOMPUTED_WHEEL_RADIUS_PI = Math.PI * WHEEL_DIAMETER;

    private final Motor steerMotor, driveMotor;

    private SwerveModuleState targetState = new SwerveModuleState();

    public SwerveModule(Motor driveMotor, Motor steerMotor) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
    }

    /**
     * SETS RAW VOLTAGE TO THE DRIVE MOTOR! UNSAFE! Only use FOR CHARACTERIZATION!
     *
     * @param voltage The voltage the drive motor receives
     */
    protected void runDriveMotorForCharacterization(double voltage) {
        driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public double getCurrent() {
        return driveMotor.getCurrent();
    }

    protected double getDriveWheelPositionRadians() {
        return 2 * Math.PI * driveMotor.getSystemPosition();
    }

    protected void logForSysId(SysIdRoutineLog log) {
        log.motor("DRIVE_MOTOR_SWERVE" + driveMotor.getDeviceID())
                .voltage(Volts.of(driveMotor.getVoltage()))
                .angularPosition(Rotations.of(driveMotor.getSystemPosition()))
                .angularVelocity(RotationsPerSecond.of(driveMotor.getSystemVelocity()));
    }

    protected void setTargetState(SwerveModuleState state, boolean shouldUseClosedLoop) {
        this.targetState = state;

        targetState.optimize(getCurrentAngle());

        targetState.speedMetersPerSecond = Optimizations.reduceSkew(targetState.speedMetersPerSecond, targetState.angle, getCurrentAngle());

        setTargetAngle(targetState.angle);
        setTargetVelocity(targetState.speedMetersPerSecond, shouldUseClosedLoop);
    }

    /**
     * The odometry thread can update itself faster than the main code loop.
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    protected SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        final int driveInputsLength = getDriveMotorInputs().threadSystemPosition.length;
        final int steerInputsLength = getSteerMotorInputs().threadSystemPosition.length;

        if (steerInputsLength != driveInputsLength || odometryUpdateIndex >= driveInputsLength) {
            return null;
        }

        return new SwerveModulePosition(
                getDriveMotorInputs().threadSystemPosition[odometryUpdateIndex] * PRECOMPUTED_WHEEL_RADIUS_PI,
                fromRotations(getSteerMotorInputs().threadSystemPosition[odometryUpdateIndex]));
    }

    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations());
    }

    protected void setTargetVelocity(double velocityMetresPerSecond, boolean shouldUseClosedLoop) {
        if (shouldUseClosedLoop) {
            final double targetVelocityRPSClosedLoop = Conversions.mpsToRps(velocityMetresPerSecond, WHEEL_DIAMETER);
            driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPSClosedLoop);
        } else {
            final double clampedVoltage = VOLTAGE_COMPENSATION_SATURATION * (velocityMetresPerSecond / MAX_SPEED_MPS);
            driveMotor.setOutput(MotorProperties.ControlMode.VOLTAGE, clampedVoltage);
        }
    }

    protected void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    protected SwerveModuleState getCurrentState() {
        return new SwerveModuleState(Conversions.rpsToMps(driveMotor.getSystemVelocity(), WHEEL_DIAMETER), getCurrentAngle());
    }

    protected SwerveModuleState getTargetState() {
        return targetState;
    }

    private Rotation2d getCurrentAngle() {
        return fromRotations(steerMotor.getSystemPosition());
    }

    private MotorInputs getSteerMotorInputs() {
        return steerMotor.getInputs();
    }
    private MotorInputs getDriveMotorInputs() {
        return driveMotor.getInputs();
    }
}
