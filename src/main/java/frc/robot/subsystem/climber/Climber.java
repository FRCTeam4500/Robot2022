package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Climber extends Subsystem, Sendable {
    public void setAngle(double angle);
    public void setOutput(double output);
}
