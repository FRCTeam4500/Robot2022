package frc.robot.subsystem.vision;

/**
 * Interface for something that gives an offset
 * This will most likely be a Vision component
 */

public interface OffsetProvider {
    public double getXOffset();
    public double getYOffset();
}
