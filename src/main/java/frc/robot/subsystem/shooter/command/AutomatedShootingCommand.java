package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderRunCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterConstants;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.command.WaitForTargetCommand;
import frc.robot.subsystem.shooter.ShooterSpinDownCommand;
import frc.robot.subsystem.shooter.ShooterSpinUpCommand;
import frc.robot.subsystem.shooter.ShooterControl;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutomatedShootingCommand extends SequentialCommandGroup {

    public AutomatedShootingCommand(Shooter shooter, Vision vision, Loader loader, ShooterControl shooterControl){
        addRequirements(shooter, loader);
        addCommands(
                new ParallelCommandGroup( //spins up the shooter and waits for the turret to find a target
                new InstantCommand(() -> {shooterControl.shooterSpeed = 1000;}),
                new ShooterSpinUpCommand(shooter, shooterControl).withTimeout(1),
                new WaitForTargetCommand(vision, ShooterConstants.maximumAllowableOffset).withTimeout(1)
                ),
                new LoaderRunCommand(loader).withTimeout(2), //shoots
                new ShooterSpinDownCommand(shooter)
        );
    }
}
