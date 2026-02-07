package frc.lib.generic.hardware.motor;

public enum MotorSignal {
    VOLTAGE(0),
    CURRENT(1),
    TEMPERATURE(2),
    CLOSED_LOOP_TARGET(3),
    POSITION(4),
    VELOCITY(5),
    ACCELERATION(6),

    /**
     * Registers both position and velocity signals together.
     * For threaded use, prefer this over separate POSITION/VELOCITY to enable latency compensation.
     */
    POSITION_AND_VELOCITY(7);

    private final int id;

    MotorSignal(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
