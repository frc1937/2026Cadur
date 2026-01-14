package frc.lib.scurve;

class PositionThirdOrderStep1 {
    public static final double DBL_EPSILON = Math.ulp(1.0);

    double v0, a0;
    double _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    double pd;
    double v0_v0;
    double a0_a0, a0_p3, a0_p4;
    double jMax_jMax;

    PositionThirdOrderStep1(double p0, double v0, double a0, double pf,
                            double vMax, double vMin, double aMax, double aMin,
                            double jMax) {
        this.v0 = v0;
        this.a0 = a0;
        this._vMax = vMax;
        this._vMin = vMin;
        this._aMax = aMax;
        this._aMin = aMin;
        this._jMax = jMax;

        pd = pf - p0;

        v0_v0 = v0 * v0;

        a0_a0 = a0 * a0;

        a0_p3 = a0 * a0_a0;
        a0_p4 = a0_a0 * a0_a0;

// max values needs to be invariant to plus minus sign change
        jMax_jMax = jMax * jMax;
    }

    private PositionResult time_all_vel(Profile profile, double vMax, double vMin, double aMax, double aMin,
                                        double jMax) {
// ACC0_ACC1_VEL
        profile.time[0] = (-a0 + aMax) / jMax;
        profile.time[1] = (a0_a0 / 2 - aMax * aMax - jMax * (v0 - vMax)) / (aMax * jMax);
        profile.time[2] = aMax / jMax;
        profile.time[3] = (3 * (a0_p4 * aMin) + 8 * aMax * aMin * (-a0_p3 + 3 * jMax * (a0 * v0)) + 6 * a0_a0 * aMin * (
                aMax * aMax - 2 * jMax * v0) - 12 *
                jMax * (aMax * aMin * (aMax * (v0 + vMax) - aMin * (vMax) - 2 * jMax * pd) + (
                aMin - aMax) * jMax * vMax * vMax + jMax * (-aMin * v0_v0))) / (
                24 * aMax * aMin * jMax_jMax * vMax);
        profile.time[4] = -aMin / jMax;
        profile.time[5] = -(-aMin * aMin - jMax * (-vMax)) / (aMin * jMax);
        profile.time[6] = profile.time[4];

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        // ACC1_VEL
        double t_acc0 = Math.sqrt(a0_a0 / (2 * jMax_jMax) + (vMax - v0) / jMax);

        profile.time[0] = t_acc0 - a0 / jMax;
        profile.time[1] = 0;
        profile.time[2] = t_acc0;
        profile.time[3] = -(-8 * aMin * (-a0_p3) - 24 * aMin * jMax * (a0 * v0) -
                12 * jMax * (
                2 * aMin * jMax * pd + aMin * aMin * (vMax) + jMax * (vMax * vMax) + aMin
                        * t_acc0 * (a0_a0 - 2 * jMax * (v0 + vMax)))) / (24 * aMin * jMax_jMax * vMax);

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        // ACC0_VEL
        double t_acc1 = Math.sqrt((vMax) / jMax);

        profile.time[0] = (-a0 + aMax) / jMax;
        profile.time[1] = (a0_a0 / 2 - aMax * aMax - jMax * (v0 - vMax)) / (aMax * jMax);
        profile.time[2] = aMax / jMax;
        profile.time[3] = (3 * a0_p4 + 8 * aMax * (-a0_p3) + 24 * aMax * jMax * (a0 * v0) + 6 * a0_a0
                * (aMax * aMax - 2 * jMax * v0) - 12 * jMax * (
                -2 * aMax * jMax * pd + aMax * aMax * (v0 + vMax) + jMax * (vMax * vMax - v0_v0) + aMax
                        * t_acc1 * (2 * (vMax) * jMax))) / (24 * aMax * jMax_jMax * vMax);
        profile.time[4] = t_acc1;
        profile.time[5] = 0;
        profile.time[6] = t_acc1;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_VEL, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

// VEL
// Solution 3/4
        profile.time[0] = t_acc0 - a0 / jMax;
        profile.time[1] = 0;
        profile.time[2] = t_acc0;
        profile.time[3] = ( - a0_p3) / (3 * jMax_jMax * vMax) + (
                a0 * v0 + (a0_a0 * t_acc0) / 2) / (jMax * vMax) - (
                v0 / vMax + 1.0) * t_acc0 - t_acc1 + pd / vMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        return new PositionResult(profile, false);
    }

    PositionResult time_acc0_acc1(Profile profile, double vMax, double vMin, double aMax,
                                  double aMin, double jMax) {
        double h1 = (3 * (-a0_p4 * aMin) + aMax * aMin * (
                8 * (a0_p3 - 0) + 3 * aMax * aMin * (aMax - aMin) - 6 * aMax * a0_a0) +
                12 * jMax * (aMax * aMin * ((aMax - 2 * a0) * v0) + aMin * a0_a0 * v0)) / (3 * (aMax - aMin) * jMax_jMax) + 4 * (- aMin * v0_v0 - 2 * aMin * aMax * pd) / (aMax - aMin);

        if (h1 >= 0) {
            h1 = Math.sqrt(h1) / 2;
            double h2 = a0_a0 / (2 * aMax * jMax) + (aMin - 2 * aMax) / (2 * jMax) - v0 / aMax;
            double h3 = - (aMax - 2 * aMin) / (2 * jMax);

// UDDU: Solution 2
            if (h2 > h1 / aMax && h3 > -h1 / aMin) {
                profile.time[0] = (-a0 + aMax) / jMax;
                profile.time[1] = h2 - h1 / aMax;
                profile.time[2] = aMax / jMax;
                profile.time[3] = 0;
                profile.time[4] = -aMin / jMax;
                profile.time[5] = h3 + h1 / aMin;
                profile.time[6] = profile.time[4];

                if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                    return new PositionResult(profile, true);
                }
            }

            // UDDU: Solution 1
            if (h2 > -h1 / aMax && h3 > h1 / aMin) {
                profile.time[0] = (-a0 + aMax) / jMax;
                profile.time[1] = h2 + h1 / aMax;
                profile.time[2] = aMax / jMax;
                profile.time[3] = 0;
                profile.time[4] = -aMin / jMax;
                profile.time[5] = h3 - h1 / aMin;
                profile.time[6] = profile.time[4];

                if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                    return new PositionResult(profile, true);
                }
            }
        }
        return new PositionResult(profile, false);
    }

    PositionResult time_all_none_acc0_acc1(Profile profile, double vMax, double vMin, double aMax,
                                           double aMin, double jMax) {

        // NONE UDDU / UDUD Strategy: t7 == 0 (equals UDDU), this one is in particular prone to numerical issues
        double h2_none = a0_a0 / (2 * jMax) + ( - v0);
        double h2_h2 = h2_none * h2_none;
        double t_min_none = a0 / jMax;
        double t_max_none = (aMax - aMin) / jMax;

        double[] polynom_none = new double[4];
        polynom_none[0] = 0;
        polynom_none[1] = -2 * (a0_a0 - 2 * jMax * (v0)) / jMax_jMax;
        polynom_none[2] = 4 * (a0_p3 - 3 * jMax * (- a0 * v0)) / (3 * jMax * jMax_jMax) - 4 * pd / jMax;
        polynom_none[3] = -h2_h2 / jMax_jMax;


// ACC0
        double h3_acc0 = (a0_a0) / (2 * aMax * jMax) + (-v0) / aMax;
        double t_min_acc0 = (aMax) / jMax;
        double t_max_acc0 = (aMax - aMin) / jMax;

        double h0_acc0 = 3 * ( - a0_p4) + 8 * (a0_p3) * aMax + 24 * aMax * jMax * (- a0 * v0)
                - 6 * a0_a0 * (aMax * aMax - 2 * jMax * v0) +
                12 * jMax * (jMax * ( - v0_v0 - 2 * aMax * pd) - aMax * aMax * ( - v0));
        double h2_acc0 = aMax * aMax;

        double[] polynom_acc0 = new double[4];
        polynom_acc0[0] = -2 * aMax / jMax;
        polynom_acc0[1] = h2_acc0 / jMax_jMax;
        polynom_acc0[2] = 0;
        polynom_acc0[3] = h0_acc0 / (12 * jMax_jMax * jMax_jMax);


        // ACC1
        double h3_acc1 = -(a0_a0) / (2 * jMax * aMin) + aMin / jMax + ( - v0) / aMin;
        double t_min_acc1 = (aMin - a0) / jMax;
        double t_max_acc1 = (aMax - a0) / jMax;

        double h0_acc1 = (a0_p4) / 4 + 2 * ( - a0_p3) * aMin / 3 + (a0_a0) * aMin * aMin / 2
                + jMax * ( a0_a0 * v0 + 2 * aMin * (jMax * pd - a0 * v0) + aMin *
                aMin * (v0) + jMax * (v0_v0));
        double h2_acc1 = a0_a0 - a0 * aMin + 2 * jMax * v0;

        double[] polynom_acc1 = new double[4];
        polynom_acc1[0] = 2 * (2 * a0 - aMin) / jMax;
        polynom_acc1[1] = (5 * a0_a0 + aMin * (aMin - 6 * a0) + 2 * jMax * v0) / jMax_jMax;
        polynom_acc1[2] = 2 * (a0 - aMin) * h2_acc1 / (jMax_jMax * jMax);
        polynom_acc1[3] = h0_acc1 / (jMax_jMax * jMax_jMax);

        polynom_acc0[0] += 4 * t_min_acc0;
        polynom_acc0[1] += (3 * polynom_acc0[0] + 6 * t_min_acc0) * t_min_acc0;
        polynom_acc0[2] += (2 * polynom_acc0[1] + (3 * polynom_acc0[0] + 4 * t_min_acc0) * t_min_acc0) * t_min_acc0;
        polynom_acc0[3] += (polynom_acc0[2] + (polynom_acc0[1] + (polynom_acc0[0] + t_min_acc0) * t_min_acc0) *
                t_min_acc0) * t_min_acc0;

        boolean polynom_acc0_has_solution = (polynom_acc0[0] < 0.0) || (polynom_acc0[1] < 0.0) || (
                polynom_acc0[2] < 0.0) || (polynom_acc0[3] <= 0.0);
        boolean polynom_acc1_has_solution = (polynom_acc1[0] < 0.0) || (polynom_acc1[1] < 0.0) || (
                polynom_acc1[2] < 0.0) || (polynom_acc1[3] <= 0.0);

        Roots.PositiveSet roots_none = Roots.solveQuartMonic(polynom_none);
        Roots.PositiveSet roots_acc0 = polynom_acc0_has_solution
                ? Roots.solveQuartMonic(polynom_acc0)
                : new Roots.PositiveSet();
        Roots.PositiveSet roots_acc1 = polynom_acc1_has_solution
                ? Roots.solveQuartMonic(polynom_acc1)
                : new Roots.PositiveSet();

        for (Double t : roots_none) {
            if (t == null || t < t_min_none || t > t_max_none) {
                continue;
            }

            // Single Newton-step (regarding pd)
            if (t > DBL_EPSILON) {
                double h1 = jMax * t * t;
                double orig = -h2_h2 / (4 * jMax * t) + h2_none * (t) + (
                        4 * a0_p3 + - 6 * a0_a0 * (2 * jMax * t) + 12 * (-a0) * jMax
                                * v0 + 3 * jMax_jMax * (-4 * pd + (h1 + 8 * v0) * t)) / (12 * jMax_jMax);
                double deriv = h2_none + 2 * v0 - a0_a0 / jMax + h2_h2 / (4 * h1) + (3 * h1) / 4;

                t -= orig / deriv;
            }

            double h0 = h2_none / (2 * jMax * t);
            profile.time[0] = h0 + t / 2 - a0 / jMax;
            profile.time[1] = 0;
            profile.time[2] = t;
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = 0;
            profile.time[6] = -h0 + t / 2 + 0 / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        for (Double t : roots_acc0) {
            if (t == null || t < t_min_acc0 || t > t_max_acc0) {
                continue;
            }

            // Single Newton step (regarding pd)
            if (t > DBL_EPSILON) {
                double h1 = jMax * t;
                double orig = h0_acc0 / (12 * jMax_jMax * t) + t * (h2_acc0 + h1 * (h1 - 2 * aMax));
                double deriv = 2 * (h2_acc0 + h1 * (2 * h1 - 3 * aMax));

                t -= orig / deriv;
            }

            profile.time[0] = (-a0 + aMax) / jMax;
            profile.time[1] = h3_acc0 - 2 * t + jMax / aMax * t * t;
            profile.time[2] = t;
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = 0;
            profile.time[6] = ( - aMax) / jMax + t;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        if (!polynom_acc1_has_solution) {
            return new PositionResult(profile, false);
        }

        for (Double t : roots_acc1) {
            if (t == null || t < t_min_acc1 || t > t_max_acc1) {
                continue;
            }

            // Double Newton step (regarding pd)
            if (t > DBL_EPSILON) {
                double h5 = a0_p3 + 2 * jMax * a0 * v0;
                double h1 = jMax * t;
                double orig = -(h0_acc1 / 2 + h1 * (
                        h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) + aMin *
                                aMin * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;
                double deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                t -= Math.min(orig / deriv, t);

                h1 = jMax * t;
                orig = -(h0_acc1 / 2 + h1 * (h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) +
                        aMin * aMin * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;

                if (Math.abs(orig) > 1e-9) {
                    deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                    t -= orig / deriv;

                    h1 = jMax * t;
                    orig = -(h0_acc1 / 2 + h1 * (
                            h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) + aMin * aMin
                                    * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;

                    if (Math.abs(orig) > 1e-9) {
                        deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                        t -= orig / deriv;
                    }
                }
            }

            profile.time[0] = t;
            profile.time[1] = 0;
            profile.time[2] = (a0 - aMin) / jMax + t;
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = h3_acc1 - (2 * a0 + jMax * t) * t / aMin;
            profile.time[6] = ( - aMin) / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        return new PositionResult(profile, false);
    }


    PositionResult time_acc1_vel_two_step(Profile profile, double vMax, double vMin, double aMax,
                                          double aMin, double jMax) {
        profile.time[0] = 0;
        profile.time[1] = 0;
        profile.time[2] = a0 / jMax;
        profile.time[3] = -( - 8 * aMin * (- a0_p3) - 24 * aMin * jMax * (a0 * v0) - 12 * jMax * (
                2 * aMin * jMax * pd + aMin * aMin * (vMax) + jMax * (vMax * vMax) + aMin
                        * a0 * (a0_a0 - 2 * jMax * (v0 + vMax)) / jMax)) / (24 * aMin * jMax_jMax * vMax);
        profile.time[4] = -aMin / jMax;
        profile.time[5] = -(-aMin * aMin + jMax * (vMax)) / (aMin * jMax);
        profile.time[6] = profile.time[4];

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        return new PositionResult(profile, false);
    }

    PositionResult

    time_acc0_two_step(Profile profile, double vMax, double vMin, double aMax,
                       double aMin, double jMax) {
// Two step
        profile.time[0] = 0;
        profile.time[1] = ( - a0_a0 + 2 * jMax * ( - v0)) / (2 * a0 * jMax);
        profile.time[2] = (a0) / jMax;
        profile.time[3] = 0;
        profile.time[4] = 0;
        profile.time[5] = 0;
        profile.time[6] = 0;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        // Three step - Removed pf
        {
            profile.time[0] = (-a0 + aMax) / jMax;
            profile.time[1] = (a0_a0 - 2 * aMax * aMax + 2 * jMax * ( - v0)) / (2 * aMax * jMax);
            profile.time[2] = (aMax) / jMax;
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = 0;
            profile.time[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        // Three step - Removed aMax
        {
            double h0 = 3 * ( - a0_a0 + 2 * jMax * (v0));
            double h2 = a0_p3 + 6 * jMax_jMax * pd;
            double h1 = Math.sqrt(
                    2 * (2 * h2 * h2 + h0 * (
                            a0_p4 + 8 * a0 * (3 * jMax_jMax * pd) - 3 * (4 * jMax_jMax * (- v0_v0))))) *
                    Math.abs(jMax) / jMax;
            profile.time[0] = ( 2 * a0_p3 + 12 * jMax_jMax * pd + h1) / (2 * jMax * h0);
            profile.time[1] = -h1 / (jMax * h0);
            profile.time[2] = (-4 * a0_p3 + 12 * jMax_jMax * pd - 12 * (- a0) * jMax *
                    v0 + h1) / (2 * jMax * h0);
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = 0;
            profile.time[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        // Three step - t=(aMax - aMin)/jMax
        {
            double t = (aMax - aMin) / jMax;

            profile.time[0] = (-a0 + aMax) / jMax;
            profile.time[1] = (a0_a0) / (2 * aMax * jMax) + ( - v0 + jMax * t * t) / aMax - 2 * t;
            profile.time[2] = t;
            profile.time[3] = 0;
            profile.time[4] = 0;
            profile.time[5] = 0;
            profile.time[6] = (0 - aMin) / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        return new PositionResult(profile, false);
    }

    PositionResult

    time_vel_two_step(Profile profile, double vMax, double vMin, double aMax,
                      double aMin, double jMax) {
        double h1 = Math.sqrt((vMax) / jMax);
        // Four step
        {
// Solution 3/4
            profile.time[0] = -a0 / jMax;
            profile.time[1] = 0;
            profile.time[2] = 0;
            profile.time[3] = ( - a0_p3) / (3 * jMax_jMax * vMax) + (a0 * v0) / (
                    jMax * vMax) - h1 + pd / vMax;
            profile.time[4] = h1;
            profile.time[5] = 0;
            profile.time[6] = h1;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        // Four step
        {
            profile.time[0] = 0;
            profile.time[1] = 0;
            profile.time[2] = a0 / jMax;
            profile.time[3] = ( - a0_p3) / (3 * jMax_jMax * vMax) + (
                    a0 * v0 + (a0_p3 / jMax) / 2) / (jMax * vMax) - (
                    v0 / vMax + 1.0) * a0 / jMax - h1 + pd / vMax;
            profile.time[4] = h1;
            profile.time[5] = 0;
            profile.time[6] = h1;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
                return new PositionResult(profile, true);
            }
        }

        return new PositionResult(profile, false);
    }

    PositionResult

    time_none_two_step(Profile profile, double vMax, double vMin, double aMax,
                       double aMin, double jMax) {
        // Two step

        double h0 = Math.sqrt((a0_a0) / 2 + jMax * ( - v0)) * Math.abs(jMax) / jMax;
        profile.time[0] = (h0 - a0) / jMax;
        profile.time[1] = 0;
        profile.time[2] = (h0) / jMax;
        profile.time[3] = 0;
        profile.time[4] = 0;
        profile.time[5] = 0;
        profile.time[6] = 0;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

// Single step

        profile.time[0] = (0 - a0) / jMax;
        profile.time[1] = 0;
        profile.time[2] = 0;
        profile.time[3] = 0;
        profile.time[4] = 0;
        profile.time[5] = 0;
        profile.time[6] = 0;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
            return new PositionResult(profile, true);
        }

        return new PositionResult(profile, false);
    }

    Profile get_profile(Profile input) {
        PositionResult result = new PositionResult(input, false);

        double vMax = pd >= 0 ? _vMax : _vMin;
        double vMin = pd >= 0 ? _vMin : _vMax;
        double aMax = pd >= 0 ? _aMax : _aMin;
        double aMin = pd >= 0 ? _aMin : _aMax;
        double jMax = pd >= 0 ? _jMax : -_jMax;

        if (Math.abs(v0) < DBL_EPSILON && Math.abs(a0) < DBL_EPSILON && Math.abs(pd) < DBL_EPSILON) {
            result = time_all_none_acc0_acc1(result.profile, vMax, vMin, aMax, aMin, jMax);
        } else {
            result = time_all_vel(result.profile, vMax, vMin, aMax, aMin, jMax);
            if (!result.has_found) result = time_all_none_acc0_acc1(result.profile, vMax, vMin, aMax, aMin, jMax);
            if (!result.has_found) result = time_acc0_acc1(result.profile, vMax, vMin, aMax, aMin, jMax);
            if (!result.has_found) result = time_all_vel(result.profile, vMin, vMax, aMin, aMax, -jMax);
            if (!result.has_found) result = time_all_none_acc0_acc1(result.profile, vMin, vMax, aMin, aMax, -jMax);
            if (!result.has_found) result = time_acc0_acc1(result.profile, vMin, vMax, aMin, aMax, -jMax);
        }

        if (!result.has_found) {
            result = time_none_two_step(result.profile, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (!result.has_found) result = time_none_two_step(result.profile, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (!result.has_found) result = time_acc0_two_step(result.profile, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (!result.has_found) result = time_acc0_two_step(result.profile, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (!result.has_found) result = time_vel_two_step(result.profile, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (!result.has_found) result = time_vel_two_step(result.profile, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (!result.has_found) result = time_acc1_vel_two_step(result.profile, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (!result.has_found) result = time_acc1_vel_two_step(result.profile, _vMin, _vMax, _aMin, _aMax, -_jMax);
        }

        return result.profile;
    }
}
