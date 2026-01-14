package frc.lib.scurve;

public class InputParameter {
    void initialize() {
        current_velocity = 0.0;
        current_acceleration = 0.0;
    }

    //! Current state
    public double current_position, current_velocity = 0, current_acceleration = 0;

    //! Target state
    public double target_position;

    public InputParameter(double current_position, double current_velocity, double current_acceleration, double target_position) {
        this.current_position = current_position;
        this.current_velocity = current_velocity;
        this.current_acceleration = current_acceleration;
        this.target_position = target_position;
    }

    InputParameter() {
        initialize();
    }

    boolean isEqual(InputParameter rhs) {
        return (current_position == rhs.current_position
                && current_velocity == rhs.current_velocity
                && current_acceleration == rhs.current_acceleration
                && target_position == rhs.target_position
        );
    }
}
