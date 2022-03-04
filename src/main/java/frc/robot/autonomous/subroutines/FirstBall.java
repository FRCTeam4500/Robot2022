package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.autonomous.Paths;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.intake.command.IntakeSetOutputCommand;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinDownCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.utility.ExtendedTrajectoryUtilities;

public class FirstBall extends SequentialCommandGroup {

    public FirstBall(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader){
        addRequirements(swerve, arm, intake, shooter);
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("FirstBall");
        Command swerveCmd = NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path);

        addCommands(
                //sets robot translation to the first of the path, then resets the robot angle to 0
                new InstantCommand(() -> swerve.resetPose(path.getInitialPose())),
                new ParallelCommandGroup(
                        new ArmSetAngleCommand(arm, ArmConstants.ARM_DOWN_ANGLE),
                        new IntakeSetOutputCommand(intake),
                        new SequentialCommandGroup(
                                new WaitCommand(0.25),
                                swerveCmd
                        )
                ).withTimeout(1),
                new InstantCommand(() -> swerve.moveFieldCentric(0, 0, 0)),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                        new IntakeRunCommand(intake, 0),
                        new ManualShootingCommand(shooter, vision, loader, new ShooterControl(24000, 1000))
                ).withTimeout(2),
                new ShooterSpinDownCommand(shooter)
        );
    }
}

