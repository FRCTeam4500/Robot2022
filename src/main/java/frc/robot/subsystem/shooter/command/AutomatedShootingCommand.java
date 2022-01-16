package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderRunCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterConstants;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.command.WaitForTargetCommand;

public class AutomatedShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;

    public AutomatedShootingCommand(Shooter shooter, Vision vision, Loader loader){
        this.shooter = shooter;
        this.vision = vision;
        addCommands(
                new ParallelCommandGroup( //spins up the shooter and waits for the turret to find a target
                new SpinUpCommand(shooter, 100d, 4d).withTimeout(1),
                new WaitForTargetCommand(vision, ShooterConstants.maximumAllowableOffset).withTimeout(1)
                ),
                new LoaderRunCommand(loader).withTimeout(2) //shoots
        );
    }
}
