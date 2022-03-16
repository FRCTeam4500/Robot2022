package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderRunConditionalCommand;
import frc.robot.subsystem.loader.command.LoaderSetOutputCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterConstants;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.command.WaitForTargetCommand;

public class ManualShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;
    private Loader loader;

    public ManualShootingCommand(Shooter shooter, Vision vision, Loader loader, ShooterControl control){
        this.shooter = shooter;
        this.vision = vision;
        this.loader = loader;
        addCommands(
                new ParallelCommandGroup( //spins up the shooter and waits for the turret to find a target
                    new ShooterSpinUpCommand(shooter, control).withTimeout(0.80),
                    new LoaderRunConditionalCommand(loader, shooter::atSpeed)
                )
        );
    }
    
    public void end(){
        loader.setOutput(0);
        shooter.setSpeed(0);
    }

    public void interrupted(){
        end();
    }
}
