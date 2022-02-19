package frc.robot.subsystem.shooter.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class ShooterControl implements Sendable {
    public double shooterSpeed = 20000;
    public double speedThreshold = 50;
    public ShooterControl(double speed, double speedThreshold){
        shooterSpeed = speed;
        this.speedThreshold = speedThreshold;
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("Shooter Speed", () -> shooterSpeed, (value) -> {shooterSpeed = value;});
        builder.addDoubleProperty("Shooter Speed Threshold", () -> speedThreshold, (value) -> {speedThreshold = value;});
    }
}
