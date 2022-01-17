package frc.robot.autonomous.routinesDriveOnly;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subRoutinesDriveOnly.FirstBall;
import frc.robot.autonomous.subRoutinesDriveOnly.WSecondPart;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class WAuto extends SequentialCommandGroup {
    public WAuto(PathFollowingSwerve swerve) {
        addCommands(
                new FirstBall(swerve),
                new WSecondPart(swerve)
        );
    }
}
