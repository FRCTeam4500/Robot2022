package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterConstants;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.command.WaitForTargetCommand;

public class AutomatedShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;

    public AutomatedShootingCommand(Shooter shooter, Vision vision){
        this.shooter = shooter;
        this.vision = vision;
        addCommands(
                new SpinUpCommand(shooter, 100d, 4d).withTimeout(1),
                new WaitForTargetCommand(vision, ShooterConstants.maximumAllowableOffset).withTimeout(1)
                //TODO: indexer stuff
        );
    }


}
