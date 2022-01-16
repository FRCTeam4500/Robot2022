package frc.robot.autonomous.subroutines;

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

/**
 * Second Part of the triangular autonomous route, to be run after FirstBall
 */

public class TriangleSecondPart extends SequentialCommandGroup {

    public TriangleSecondPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("TriangleSecondPart");
        addCommands(
                new ParallelCommandGroup(
                        new FollowDottedTrajectoryCommand(swerve, path, ExtendedTrajectoryUtilities.createBasicController(1,1,1,4,1)),
                        new IntakeSetSpeedCommand(intake, IntakeConstants.intakeRunSpeed)
                ),
                new WaitCommand(0.25),
                new ParallelCommandGroup(
                        new AutomatedShootingCommand(shooter,vision).withTimeout(2),
                        new ArmSetAngleCommand(arm, 0)
                )
        );
    }
}