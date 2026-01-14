package frc.lib.generic.hardware.encoder;

import frc.lib.generic.hardware.encoder.hardware.GenericCanCoder;
import frc.lib.generic.hardware.encoder.hardware.SimulatedCanCoder;
import frc.robot.GlobalConstants;

import static frc.robot.GlobalConstants.CURRENT_MODE;

public class EncoderFactory {
    public static Encoder createCanCoder(String name, int port) {
        if (CURRENT_MODE == GlobalConstants.Mode.REAL) {
            return new GenericCanCoder(name, port);
        }

        if (CURRENT_MODE == GlobalConstants.Mode.SIMULATION) {
            return new SimulatedCanCoder(name);
        }

        return new Encoder(name);
    }
}
