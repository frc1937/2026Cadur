package frc.lib.scurve;

//! Output of the Scurve algorithm
public class OutputParameter {
    //! Current trajectory
    public Trajectory trajectory = new Trajectory();

    // Current kinematic state
    public double new_position, new_velocity, new_acceleration, new_jerk;

    //! Current time on trajectory
    public double time = 0;

    public InputParameter update_input_from_output(InputParameter input)  {
        input.current_position = new_position;
        input.current_velocity = new_velocity;
        input.current_acceleration = new_acceleration;

        return input;
    }
}