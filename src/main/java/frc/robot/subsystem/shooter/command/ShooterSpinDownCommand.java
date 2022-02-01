package frc.robot.subsystem.shooter.command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.shooter.Shooter;

public class ShooterSpinDownCommand extends InstantCommand {
    private Shooter shooter;
    public ShooterSpinDownCommand(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setSpeed(0);
    }
}
