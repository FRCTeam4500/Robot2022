package frc.robot.autonomous.routinesDriveOnly;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subRoutinesDriveOnly.FirstBall;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

public class FirstBallAuto extends SequentialCommandGroup {
    public FirstBallAuto(PathFollowingSwerve swerve){
        addCommands(
                new FirstBall(swerve)
        );
    }
}
