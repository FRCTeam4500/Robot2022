package frc.robot.subsystem.intake;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Intake extends Subsystem, Sendable {
    public void setOutput(double output);
}

