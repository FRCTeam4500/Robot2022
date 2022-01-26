package frc.robot.subsystems.shooter;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        return (Math.abs(shooterControl.shooterSpeed - shooter.getSpeed()) < shooterControl.speedThreshold);
    }

    
}
