package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderRunCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.util.ShooterParameterCalculator;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.util.VisionDistanceCalculator;
import frc.robot.subsystem.loader.command.LoaderRunConditionalCommand;

public class AutomatedShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;
    private Loader loader;

    private double targetSpeed;
    private double targetAngle;

    public AutomatedShootingCommand(Shooter shooter, Vision vision, Loader loader){
        this.shooter = shooter;
        this.vision = vision;
        this.loader = loader;
        addCommands(
            new ParallelCommandGroup(
                new ShooterContinuousRunCommand(shooter, () -> {return ShooterParameterCalculator.getSpeed(VisionDistanceCalculator.calculateDistance(vision));}),
                new LoaderRunConditionalCommand(loader, shooter::atSpeed)) //shoots
        );
    }
    public void end(){
        loader.setOutput(0);
        shooter.setSpeed(0);
    }
}
