package frc.robot.subsystem.camera;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Camera extends Subsystem, Sendable {
    public void createSimpleStream();
}
