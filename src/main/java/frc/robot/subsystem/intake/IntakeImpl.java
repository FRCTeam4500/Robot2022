package frc.robot.subsystem.intake;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.OutputComponent;

public class IntakeImpl implements Intake{
    private OutputComponent motor;
    private double targetOutput;
    public IntakeImpl(OutputComponent motor){
        this.motor = motor;
    }

    public void setOutput(double output){
        targetOutput = output;
        motor.setOutput(output);
    }

    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("Running", () -> !(targetOutput == 0), null);
        builder.addDoubleProperty("Output", motor::getOutput, null);
    }
}
