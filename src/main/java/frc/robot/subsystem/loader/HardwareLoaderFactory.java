package frc.robot.subsystem.loader;

import frc.robot.component.OutputComponent;
import frc.robot.component.hardware.VictorSPXComponent;

public class HardwareLoaderFactory {
    public static Loader makeLoader(){
        VictorSPXComponent motor = new VictorSPXComponent(13);
        return new LoaderImpl(motor);
    }
}
