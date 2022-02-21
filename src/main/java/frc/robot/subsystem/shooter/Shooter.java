package frc.robot.subsystem.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Shooter extends Subsystem, Sendable {
    public void setSpeed(double speed);
    public double getSpeed();
    public boolean atSpeed();
}
