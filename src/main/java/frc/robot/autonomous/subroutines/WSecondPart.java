package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import frc.robot.subsystem.arm.ArmConstants;

public class WSecondPart extends SequentialCommandGroup {
    public WSecondPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("WSecondPart");
        FollowDottedTrajectoryCommand swervePathCommand = new FollowDottedTrajectoryCommand(
                swerve,
                path,
                ExtendedTrajectoryUtilities.createBasicController(1,1,1,4,1));
        swervePathCommand.setRotation(true);
        addCommands( //whether or not we need to add a command to turn the robot depends on how fast it can turn
                new ParallelCommandGroup( //runs the path and runs the intake
                        swervePathCommand,
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new IntakeRunCommand(intake).withTimeout(1), //Run intake for 1 second
                                new WaitCommand(1),
                                new IntakeRunCommand(intake, IntakeConstants.intakeRunSpeed).withTimeout(1)
                        )
                ),
                new ParallelCommandGroup(
                        new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE),
                        new AutomatedShootingCommand(shooter, vision, loader).withTimeout(2)
                )
        );
    }

}
