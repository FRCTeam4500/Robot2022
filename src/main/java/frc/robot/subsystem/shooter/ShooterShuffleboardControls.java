package frc.robot.subsystems.shooter;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Robot;

public class ShooterShuffleboardControls implements NTSendable {
    private ShooterControl control;

    public ShooterShuffleboardControls(ShooterControl control){
        this.control = control;
    }

    public void initSendable(NTSendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("Shooter Speed", () -> control.shooterSpeed, (value) -> {control.shooterSpeed = value;});
        builder.addDoubleProperty("Shooter Speed Threshold", () -> control.speedThreshold, (value) -> {control.speedThreshold = value;});
        SendableRegistry.enableLiveWindow(this);
    }
}
