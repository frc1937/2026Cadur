package frc.lib.scurve;

public enum Result {
    Working(0), ///< The trajectory is calculated normally
    Finished(1), ///< The trajectory has reached its final position
    ErrorExecutionTimeCalculation(-110);

    public int result;

    Result(int i) {
    }

    public int getResult() {
        return result;
    }
}
