package frc.robot.subsystem.vision;

import frc.robot.component.VisionComponent;

public class VisionImpl implements Vision{
    private  VisionComponent visionComponent;

    public VisionImpl(VisionComponent camera){
        visionComponent = camera;
    }

    public boolean hasValidTargets(){
        return visionComponent.hasValidTargets();
    }

    public double getHorizontalOffsetFromCrosshair(){
        return visionComponent.getHorizontalOffsetFromCrosshair();
    }

    public double getVerticalOffsetFromCrosshair(){
        return visionComponent.getVerticalOffsetFromCrosshair();
    }

    public double getTargetArea(){
        return visionComponent.getTargetArea();
    }

    public double getSkew(){
        return visionComponent.getSkew();
    }

}
