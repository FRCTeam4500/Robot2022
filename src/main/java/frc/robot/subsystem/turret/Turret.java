package frc.robot.subsystem.turret;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Turret extends Subsystem, Sendable {
    public void setAngle(double angle);
    public double getAngle();
    public void setOutput(double output);
}
