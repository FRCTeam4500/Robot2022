/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.component.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.component.TransformSupplier;
import frc.robot.component.VisionComponent;
import frc.robot.utility.Transform3D;

/**
 * An {@link VisionComponent} wrapper for a Limelight vision processor.
 */
public class LimelightVisionComponent implements VisionComponent, TransformSupplier {

    private NetworkTable table;

    /**
     * Creates a new VisionComponent component, which is essentially a wrapper
     * around the networktable entries modified by a Limelight.
     */
    public LimelightVisionComponent() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public boolean hasValidTargets() {
        double value = getEntry("tv");
        if (value == 1) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public double getHorizontalOffsetFromCrosshair() {
        return Math.toRadians(getEntry("tx"));
    }

    @Override
    public double getVerticalOffsetFromCrosshair() {
        return Math.toRadians(getEntry("ty"));
    }

    @Override
    public double getTargetArea() {
        return getEntry("ta");
    }

    @Override
    public double getSkew() {
        double rawDegrees = getEntry("ts");
        double adjustedDegrees;
        if (Math.abs(rawDegrees) < 45) {
            adjustedDegrees = -rawDegrees;
        } else {
            adjustedDegrees = -(90 + rawDegrees);
        }
        return Math.toRadians(adjustedDegrees);
    }

    @Override
    public void setCameraMode(CameraMode mode) {
        if (mode == CameraMode.DriverCamera) {
            setEntry("camMode", 1);
        } else if (mode == CameraMode.VisionProcessor) {
            setEntry("camMode", 0);
        } // Maybe add exception for bad entry
    }

    @Override
    public void setPipeline(int index) {
        setEntry("pipeline", index);
    }

    private double getEntry(String key) {
        return table.getEntry(key).getDouble(0);
    }

    private void setEntry(String key, Number value) {
        table.getEntry(key).setNumber(value);
    }

    private double[] getCameraTranslationEntry() {
        return table.getEntry("camtran").getDoubleArray(new double[6]);
    }

    @Override
    public Transform3D getTransform() {
        double[] translation = getCameraTranslationEntry();
        return new Transform3D(translation[0], translation[1],
                translation[2], translation[3], translation[4],
                translation[5]);
    }
}
