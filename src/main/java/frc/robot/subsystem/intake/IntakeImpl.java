package frc.robot.subsystem.intake;

import frc.robot.component.OutputComponent;

public class IntakeImpl implements Intake {
    
    private OutputComponent motor;

    public IntakeImpl(OutputComponent motor) {
        this.motor = motor;
    }

    public void setOutput(double output) {
        motor.setOutput(output);
    }

}
