package frc.robot.subsystem.loader;

import frc.robot.component.OutputComponent;

public class HardwareLoaderFactory {
    public Loader makeLoader(){
        OutputComponent motor = null; //TODO: add motor
        return new LoaderImpl(motor);
    }
}
