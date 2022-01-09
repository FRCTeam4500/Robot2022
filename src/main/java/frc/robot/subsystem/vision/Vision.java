package frc.robot.subsystem.vision;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for vision subsystems
 */

public interface Vision extends OffsetProvider, Subsystem {
    boolean hasValidTargets();

    double getHorizontalOffsetFromCrosshair();

    double getVerticalOffsetFromCrosshair();

    double getTargetArea();

    double getSkew();
}
