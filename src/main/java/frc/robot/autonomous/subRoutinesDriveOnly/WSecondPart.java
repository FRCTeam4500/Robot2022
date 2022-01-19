package frc.robot.autonomous.subRoutinesDriveOnly;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.utility.ExtendedTrajectoryUtilities;

public class WSecondPart extends SequentialCommandGroup {
    public WSecondPart(PathFollowingSwerve swerve) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("WSecondPart");
        FollowDottedTrajectoryCommand swervePathCommand = new FollowDottedTrajectoryCommand(
                swerve,
                path,
                ExtendedTrajectoryUtilities.createBasicController(1,1,1,4,1));
        swervePathCommand.setRotation(true);
        addCommands( //whether or not we need to add a command to turn the robot depends on how fast it can turn
                new ParallelCommandGroup( //runs the path and runs the intake
                        swervePathCommand,
                        new WaitCommand(1)
                ),
                new WaitCommand(1)
        );
    }

}
