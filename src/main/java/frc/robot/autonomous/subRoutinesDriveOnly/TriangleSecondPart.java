package frc.robot.autonomous.subRoutinesDriveOnly;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.utility.ExtendedTrajectoryUtilities;

/**
 * Second Part of the triangular autonomous route, to be run after FirstBall
 */

public class TriangleSecondPart extends SequentialCommandGroup {

    public TriangleSecondPart(PathFollowingSwerve swerve) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("TriangleSecondPart");
        addCommands(
                new ParallelCommandGroup(
                        new FollowDottedTrajectoryCommand(swerve, path, ExtendedTrajectoryUtilities.createBasicController(1,1,1,4,1))
                )
        );
    }
}