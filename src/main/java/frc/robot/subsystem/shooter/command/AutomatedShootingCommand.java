package frc.robot.subsystem.shooter.command;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderRunCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterParameterCalculator;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.VisionDistanceCalculator;
import frc.robot.subsystem.vision.command.WaitForTargetCommand;

public class AutomatedShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;

    private double targetSpeed;
    private double targetAngle;

    public AutomatedShootingCommand(Shooter shooter, Vision vision, Loader loader){
        this.shooter = shooter;
        this.vision = vision;
        addCommands(
                new ParallelCommandGroup( //spins up the shooter and waits for the turret to find a target
                        /**
                         * This SpinUpCommand pre-spins up the turret to a medium speed,
                         * in order to reduce the amount of time spent spinning up after the correct speed is found
                         * TODO: change the speed to somewhere in the middle of the possible speeds given by the parameter calculator
                         */
                        new SpinUpCommand(shooter, 100d, 4d).withTimeout(0.2),
                        new WaitForTargetCommand(vision).withTimeout(1) //all of these timeouts will be adjusted later
                ),
                new InstantCommand(() -> { //Calculates the distance to the target and gets the corresponding speed and angle values
                    double distance = VisionDistanceCalculator.calculateDistance(vision);
                    targetSpeed = ShooterParameterCalculator.getSpeed(distance);

                }),
                new SpinUpCommand(shooter, targetSpeed).withTimeout(1),
                new LoaderRunCommand(loader).withTimeout(0.5), //shoots
                new InstantCommand(() -> shooter.setSpeed(0)) //spin down the shooter
        );
    }
}
