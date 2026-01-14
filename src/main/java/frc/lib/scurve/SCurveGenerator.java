package frc.lib.scurve;

import static frc.lib.scurve.Result.Finished;
import static frc.lib.scurve.Result.Working;

//! Main interface for the SCurve algorithm
public class SCurveGenerator {
    //! Current input, only for comparison for recalculation
    private InputParameter current_input;

    //! Kinematic constraints
    public double max_velocity, max_acceleration, max_jerk;

    //! Flag that indicates if the current_input was properly initialized
    private boolean current_input_initialized = false;
    
    //! Calculator for new trajectories
    public TargetCalculator calculator = new TargetCalculator();

    //! Time step between updates (cycle time) in [s]
    public double delta_time;

    public SCurveGenerator(double delta_time, double max_velocity, double max_acceleration, double max_jerk) {
        this.delta_time = delta_time ;

        this.max_velocity =  max_velocity;
        this.max_acceleration = max_acceleration;
        this.max_jerk = max_jerk;
    }

    //! Reset the instance (e.g. to force a new calculation in the next update)
    public void reset() {
        current_input_initialized = false;
    }

    //! Calculate a new trajectory for the given input and check for interruption
    private UpdateResult calculate(InputParameter input, OutputParameter output) {
        return calculator.calculate(input, output, max_velocity, max_acceleration, max_jerk);
    }

    //! Get the next output state (with step delta_time) along the calculated trajectory for the given input
    public UpdateResult update(InputParameter input, OutputParameter output) {
        if (!current_input_initialized || !input.isEqual(current_input)) {
            UpdateResult calculate_result = calculate(input, output);

            output.trajectory = calculate_result.output_parameter.trajectory;

            current_input = calculate_result.input_parameter;
            current_input_initialized = true;
            output.time = 0.0;
        }

        output.time += delta_time;
        double[] newResults = output.trajectory.at_time(
                output.time);

        output.new_position = newResults[0];
        output.new_velocity = newResults[1];
        output.new_acceleration = newResults[2];
        output.new_jerk = newResults[3];

        current_input = output.update_input_from_output(current_input);

        if (output.time > output.trajectory.get_duration()) {
            return new UpdateResult(current_input, output, Finished);
        }

        return new UpdateResult(current_input, output, Working);
    }
}
