package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;
import frc.robot.subsystem.intake.command.IntakeSetSpeedCommand;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ExtendedTrajectoryUtilities;

public class WSecondPart extends SequentialCommandGroup {
    public WSecondPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("WSecondPart");
        FollowDottedTrajectoryCommand swervePathCommand = new FollowDottedTrajectoryCommand(
                swerve,
                path,
                ExtendedTrajectoryUtilities.createBasicController(1,1,1,4,1));
        swervePathCommand.setRotation(true);
        addCommands( //whether or not i need to add a command to turn the robot depends on how fast it can turn
                new ParallelCommandGroup( //runs the path and runs the intake
                        swervePathCommand,
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                new IntakeSetSpeedCommand(intake, IntakeConstants.intakeRunSpeed),
                                new WaitCommand(1),
                                new IntakeSetSpeedCommand(intake, 0),
                                new WaitCommand(1),
                                new IntakeSetSpeedCommand(intake, IntakeConstants.intakeRunSpeed),
                                new WaitCommand(1)
                        )
                ),
                new ParallelCommandGroup(
                        new IntakeSetSpeedCommand(intake, 0),
                        new ArmSetAngleCommand(arm, 0),
                        new AutomatedShootingCommand(shooter, vision)
                )
        );
    }

}
