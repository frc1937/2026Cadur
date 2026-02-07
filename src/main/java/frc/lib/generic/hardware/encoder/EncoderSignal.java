package frc.lib.generic.hardware.encoder;

public enum EncoderSignal {
    POSITION(0),
    VELOCITY(1),

    /**
     * Registers both position and velocity signals together.
     * For threaded use, prefer this over separate POSITION/VELOCITY to enable latency compensation.
     */
    POSITION_AND_VELOCITY(2);

    final int id;

    EncoderSignal(int id) {
        this.id = id;
    }

    public int getId() {
        return id;
    }
}
