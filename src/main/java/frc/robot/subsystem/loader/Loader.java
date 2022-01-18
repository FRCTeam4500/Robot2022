package frc.robot.subsystem.loader;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Loader extends Subsystem { //TODO: add a default loader command which uses ball sensors
    public void setSpeed(double speed);
}
