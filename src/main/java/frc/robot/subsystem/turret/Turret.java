package frc.robot.subsystem.turret;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Turret extends Subsystem {
    void setAngle(double angle);
    double getAngle();
}
