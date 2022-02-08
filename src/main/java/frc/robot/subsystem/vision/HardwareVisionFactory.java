package frc.robot.subsystem.vision;

import frc.robot.component.hardware.LimelightVisionComponent;

public class HardwareVisionFactory {
    public static Vision makeVision(){
        Vision vision = new VisionImpl(new LimelightVisionComponent());
        return vision;
    }
}
