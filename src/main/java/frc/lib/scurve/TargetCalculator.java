package frc.lib.scurve;

import static frc.lib.scurve.Result.Working;
import static frc.lib.scurve.Utilities.back;

public class TargetCalculator {
    public UpdateResult calculate(InputParameter input, OutputParameter output_parameter, double max_velocity, double max_acceleration, double max_jerk) {
        output_parameter.trajectory.profile.set_boundary(input.current_position, input.current_velocity,
                input.current_acceleration, input.target_position);

        final PositionThirdOrderStep1 step1 = new PositionThirdOrderStep1(
                output_parameter.trajectory.profile.p[0], output_parameter.trajectory.profile.v[0],
                output_parameter.trajectory.profile.a[0],

                output_parameter.trajectory.profile.pf, max_velocity, -max_velocity,
                max_acceleration, -max_acceleration, max_jerk
        );

        output_parameter.trajectory.profile = step1.get_profile(output_parameter.trajectory.profile);

        if (back(output_parameter.trajectory.profile.totalTime) > output_parameter.trajectory.duration)
            output_parameter.trajectory.duration = back(output_parameter.trajectory.profile.totalTime);

        return new UpdateResult(input, output_parameter, Working);
    }
}

