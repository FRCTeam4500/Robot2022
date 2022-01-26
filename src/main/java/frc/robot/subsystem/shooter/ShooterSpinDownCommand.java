package frc.robot.subsystems.shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
