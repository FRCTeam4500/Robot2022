package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ExtendedTrajectoryUtilities;

public class FirstBall extends SequentialCommandGroup {

    public FirstBall(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader){
        addRequirements(swerve, arm, intake, shooter);
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("FirstBall");
        FollowDottedTrajectoryCommand swerveCmd = new FollowDottedTrajectoryCommand(
                swerve, path,
                ExtendedTrajectoryUtilities.createBasicController(1,1,1, 4, 1));
        swerveCmd.setRotation(true);
        addCommands(
                new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
                new ParallelCommandGroup(
                        new ArmSetAngleCommand(arm, ArmConstants.armDownAngle),
                        new IntakeRunCommand(intake),
                        swerveCmd
                ),
                new ParallelCommandGroup(
                        new AutomatedShootingCommand(shooter, vision, loader).withTimeout(2)
                )
        );
    }
}

