package frc.robot.subsystem.loader;

import frc.robot.component.OutputComponent;

public class LoaderImpl implements Loader{
    private OutputComponent motor;
    public LoaderImpl(OutputComponent motor){
        this.motor = motor;
    }

    public void setOutput(double output){
        motor.setOutput(output);
    }
}
