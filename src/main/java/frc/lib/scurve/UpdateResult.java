package frc.lib.scurve;

public class UpdateResult {
    public InputParameter input_parameter;
    public OutputParameter output_parameter;
    public Result result;

    public UpdateResult(InputParameter input_parameter, OutputParameter output_parameter, Result result) {
        this.input_parameter = input_parameter;
        this.output_parameter = output_parameter;
        this.result = result;
    }
}