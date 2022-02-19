package frc.robot.subsystem.loader;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Loader extends Subsystem, Sendable { //TODO: add a default loader command which uses ball sensors
    public void setOutput(double speed);
}
