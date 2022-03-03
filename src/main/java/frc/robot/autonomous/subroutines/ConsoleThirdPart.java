package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryWithEndRotationOffsetCommand;
import frc.robot.utility.ExtendedTrajectoryUtilities;

import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.loader.Loader;

public class ConsoleThirdPart extends SequentialCommandGroup{

    public ConsoleThirdPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader){
        Trajectory path1 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleFirstPart");
        Trajectory path2 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleSecondPart");


    addCommands(
            new ParallelCommandGroup(
      new FollowDottedTrajectoryWithEndRotationOffsetCommand(swerve, path1, ExtendedTrajectoryUtilities.createBasicController(1, 1, 1, 4, 3),
              new Rotation2d(swerve.getRobotAngle() - path1.getStates().get(path1.getStates().size() - 1).poseMeters.getRotation().getRadians())),
                    new IntakeRunCommand(intake)
                    ).withTimeout(2),
      new WaitCommand(1),

      new ParallelCommandGroup(
      new FollowDottedTrajectoryCommand(swerve, path2, ExtendedTrajectoryUtilities.createBasicController(1, 1, 1, 4, 3)),
              new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE)
      ).withTimeout(5),
            new ManualShootingCommand(shooter, vision, loader, new ShooterControl(25000, 1000))

    );
    }
    
}
