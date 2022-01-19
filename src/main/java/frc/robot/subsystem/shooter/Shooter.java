package frc.robot.subsystem.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Shooter extends Subsystem {
    public void setSpeed(double speed);
    public double getSpeed();
    public void setAngle(double angle);
    public double getAngle();
}
