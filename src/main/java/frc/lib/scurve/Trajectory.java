package frc.lib.scurve;

import static frc.lib.scurve.Utilities.back;
import static frc.lib.scurve.Utilities.integrate;

public class Trajectory {
    Profile profile = new Profile();

    double duration = 0.0;

    double[] state_to_integrate_from(double time)  {
        double[] result;

        if (time >= duration) {
            // Keep ant acceleration
            double t_diff = time - back(profile.totalTime);
            var integrated = integrate(t_diff, back(profile.p), back(profile.v), back(profile.a), 0.0);
            result = new double[]{integrated[0], integrated[1], integrated[2], 0.0};
            return result;
        }

        double t_diff_dof = time;

        if (t_diff_dof >= back(profile.totalTime)) {
            // Keep ant acceleration
            var integrated = integrate(t_diff_dof - back(profile.totalTime), back(profile.p), back(profile.v), back(profile.a), 0.0);
            result = new double[]{integrated[0], integrated[1], integrated[2], 0.0};
            return result;
        }

        int index_dof = 0;
        while (index_dof < profile.totalTime.length && profile.totalTime[index_dof] <= t_diff_dof) {
            index_dof++;
        }

        if (index_dof > 0) {
            t_diff_dof -= profile.totalTime[index_dof - 1];
        }

        var integrated = integrate(t_diff_dof, profile.p[index_dof], profile.v[index_dof], profile.a[index_dof], profile.j[index_dof]);
        result = new double[]{integrated[0], integrated[1], integrated[2], profile.j[index_dof]};
        return result;
    }

    //! Get the kinematic state, the jerk, and the section at a given time without vectors for a single DoF
    public double[] at_time(double time)  {
        return state_to_integrate_from(time);
    }

    //! Get the duration of the (synchronized) trajectory
    public double get_duration()  {
        return duration;
    }
}