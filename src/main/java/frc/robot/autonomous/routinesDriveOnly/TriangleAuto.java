package frc.robot.autonomous.routinesDriveOnly;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subRoutinesDriveOnly.FirstBall;
import frc.robot.autonomous.subRoutinesDriveOnly.TriangleSecondPart;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class TriangleAuto extends SequentialCommandGroup {
    public TriangleAuto(PathFollowingSwerve swerve) {
        addCommands(
                new FirstBall(swerve),
                new TriangleSecondPart(swerve)
        );
    }
}
