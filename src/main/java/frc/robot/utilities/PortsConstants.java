package frc.robot.utilities;

/**
 * Class containing constants for ports on the robot.
 * This is useful for keeping track of which ports are used, so no port is used twice.
 */
public class PortsConstants {
    public static int LEDSTRIP_PORT_PWM = 9;

    public static class SwervePorts {
        public static final int FR_STEER_MOTOR_PORT = 1;
        public static final int FL_STEER_MOTOR_PORT = 2;
        public static final int RL_STEER_MOTOR_PORT = 3;
        public static final int RR_STEER_MOTOR_PORT = 4;

        public static final int FR_DRIVE_MOTOR_PORT = 1;
        public static final int FL_DRIVE_MOTOR_PORT = 2;
        public static final int RL_DRIVE_MOTOR_PORT = 3;
        public static final int RR_DRIVE_MOTOR_PORT = 4;

        public static final int FR_STEER_ENCODER_PORT = 1;
        public static final int FL_STEER_ENCODER_PORT = 2;
        public static final int RL_STEER_ENCODER_PORT = 3;
        public static final int RR_STEER_ENCODER_PORT = 4;

        public static final int GYRO_PORT = 30;
    }

    public static class TurretPorts {
        public static final int TURRET_MOTOR_PORT = 10;
        public static final int TURRET_ENCODER_PORT = 11;
    }

    public static class IntakePorts {
        public static final int INTAKE_ROLLER_MOTOR_PORT = 12;
        public static final int INTAKE_EXTENSION_MOTOR_PORT = 13;
    }

    public static class HoodPorts {
        public static final int HOOD_MOTOR_PORT = 13;
    }

    public static class FlywheelPort {
        public static final int LEFT_FLYWHEEL_PORT = 14;
        public static final int RIGHT_FLYWHEEL_PORT = 15;
    }

    public static class RevolverPorts {
        public static final int REVOLVER_MOTOR_PORT = 16;
    }

    public static class KickerPorts {
        public static final int KICKER_MOTOR_PORT = 17;
    }
}