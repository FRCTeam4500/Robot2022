package frc.robot.autonomous.subRoutinesDriveOnly;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.utility.ExtendedTrajectoryUtilities;

public class FirstBall extends SequentialCommandGroup {

    public FirstBall(PathFollowingSwerve swerve){
        addRequirements(swerve);
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("FirstBall");
        FollowDottedTrajectoryCommand swerveCmd = new FollowDottedTrajectoryCommand(
                swerve, path,
                ExtendedTrajectoryUtilities.createBasicController(1,1,1, 4, 1));
        swerveCmd.setRotation(true);
        addCommands(
                new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
                new ParallelCommandGroup(
                        swerveCmd
                )
        );
    }
}

