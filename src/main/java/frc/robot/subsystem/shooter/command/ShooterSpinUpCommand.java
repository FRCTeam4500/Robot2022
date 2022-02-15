package frc.robot.subsystem.shooter.command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.util.ShooterControl;

public class ShooterSpinUpCommand extends CommandBase{
    private Shooter shooter;
    private ShooterControl shooterControl;
    public ShooterSpinUpCommand(Shooter shooter, ShooterControl shooterControl) {
        this.shooter = shooter;
        this.shooterControl = shooterControl;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setSpeed(shooterControl.shooterSpeed);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        //return (Math.abs(shooterControl.shooterSpeed - shooter.getSpeed()) < shooterControl.speedThreshold);
        return false;
    }

    
}
