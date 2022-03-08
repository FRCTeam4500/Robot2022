package frc.robot.subsystem.turret;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Turret extends Subsystem, Sendable {
    void setAngle(double angle); //TODO: change back later, temp fix
    double getAngle();
    void setOutput(double output);
    void setOffset(double offset);
    double getOffset();
}
