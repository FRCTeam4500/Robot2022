package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.intake.command.IntakeSetOutputCommand;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
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
import frc.robot.utility.PolarVelocityCalculator;

public class ConsoleThirdPart extends SequentialCommandGroup{

    public ConsoleThirdPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader, PolarVelocityCalculator calculator){
        Trajectory path1 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleFirstPart");
        Trajectory path2 = ExtendedTrajectoryUtilities.getDeployedTrajectory("ConsoleSecondPart");


    addCommands(
            new ParallelCommandGroup(
                    NewTrajectoryUtilities.generateSwerveControllerCommand(swerve,path1),
                    new IntakeSetOutputCommand(intake)
                    ).withTimeout(3),
      new WaitCommand(1),
      new ParallelCommandGroup(
              NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path2, true),
              new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE),
              new IntakeSetOutputCommand(intake, 0)
      ).withTimeout(3),
            new AutomatedShootingCommand(shooter, vision, loader, calculator)
    );
    }
    
}
