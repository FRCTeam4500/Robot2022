package frc.robot.subsystem.loader;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.OutputComponent;

public class LoaderImpl implements Loader{
    private OutputComponent motor;
    private double targetOutput;
    public LoaderImpl(OutputComponent motor){
        this.motor = motor;
    }

    public void setOutput(double output){
        targetOutput = output;
        motor.setOutput(output);
    }

    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("Running", () -> !(targetOutput == 0), null);
        builder.addDoubleProperty("Output", motor::getOutput, null);
        builder.addDoubleProperty("Target output", () -> targetOutput, null);
    }
}
