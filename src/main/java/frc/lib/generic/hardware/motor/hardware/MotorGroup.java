package frc.lib.generic.hardware.motor.hardware;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorConfiguration;
import frc.lib.generic.hardware.motor.MotorProperties;

public class MotorGroup extends Motor {
    private final Motor[] motors;

    public MotorGroup(String name, Motor... motors) {
        super(name);
        this.motors = motors;
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output) {
        for(Motor motor : motors) {
            motor.setOutput(controlMode, output);
        }
    }

    @Override
    public void setOutput(MotorProperties.ControlMode controlMode, double output, double feedforward) {
        for(Motor motor : motors) {
            motor.setOutput(controlMode, output, feedforward);
        }
    }

    @Override
    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        for(Motor motor : motors) {
            motor.setIdleMode(idleMode);
        }
    }

    @Override
    public void stopMotor() {
        for(Motor motor : motors) {
            motor.stopMotor();
        }
    }

    @Override
    public void setMotorEncoderPosition(double position) {
        for(Motor motor : motors) {
            motor.setMotorEncoderPosition(position);
        }
    }

    @Override
    public double getClosedLoopTarget() {
        return motors[0].getClosedLoopTarget();
    }

    @Override
    public boolean configure(MotorConfiguration configuration) {
        boolean success = true;

        for(Motor motor : motors) {
            success = motor.configure(configuration);
        }

        return success;
    }
}
