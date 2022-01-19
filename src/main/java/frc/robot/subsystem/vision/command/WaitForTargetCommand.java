package frc.robot.subsystem.vision.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.vision.Vision;

/**
 * Command which does nothing except wait for the vision to align with a target.
 * Best used with timeout
 */

public class WaitForTargetCommand extends CommandBase {
    private double maxOffset;
    private Vision vision;

    public WaitForTargetCommand(Vision vision, double maximumAllowableOffset){
        maxOffset = maximumAllowableOffset;
        this.vision = vision;
    }

    public boolean isFinished(){
        return Math.abs(vision.getHorizontalOffsetFromCrosshair()) <= maxOffset; //command ends when vision aligns with its target
    }
}
