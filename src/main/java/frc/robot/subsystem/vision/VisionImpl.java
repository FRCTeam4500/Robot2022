package frc.robot.subsystem.vision;

import frc.robot.component.VisionComponent;

public class VisionImpl implements Vision{
    private  VisionComponent visionComponent;
    private double height;
    private double angle;
    private double previousOffset = 0;

    public VisionImpl(VisionComponent camera, double height, double angle){
        visionComponent = camera;
        this.height = height;
        this.angle = angle;
    }

    public boolean hasValidTargets(){
        return visionComponent.hasValidTargets();
    }

    public double getHorizontalOffsetFromCrosshair(){
        double currentOff = visionComponent.getHorizontalOffsetFromCrosshair();
        double avgOff = (currentOff + previousOffset) / 2;
        previousOffset = currentOff;
        return avgOff;

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

    public double getVisionAngle(){
        return angle;
    }

    public double getVisionHeight(){
        return height;
    }
}
