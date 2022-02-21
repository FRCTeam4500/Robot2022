package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.utility.ExtendedTrajectoryUtilities;

import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.loader.Loader;

public class Console extends SequentialCommandGroup{

    public Console(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader){
        Trajectory path1 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleFirstPart");
        Trajectory path2 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleSecondPart");


    addCommands(
      new FollowDottedTrajectoryCommand(swerve, path1, ExtendedTrajectoryUtilities.createBasicController(1, 1, 1, 4, 1)),  
      new FollowDottedTrajectoryCommand(swerve, path2, ExtendedTrajectoryUtilities.createBasicController(1, 1, 1, 4, 1))

    );
    }
    
}
