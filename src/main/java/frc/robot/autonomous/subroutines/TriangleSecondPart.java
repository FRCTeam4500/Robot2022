package frc.robot.autonomous.subroutines;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.NewTrajectoryUtilities;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinDownCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryWithEndRotationOffsetCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowTrajectoryCommand;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ExtendedTrajectoryUtilities;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.utility.PolarVelocityCalculator;

/**
 * Second Part of the triangular autonomous route, to be run after FirstBall
 */

public class TriangleSecondPart extends SequentialCommandGroup {

    public TriangleSecondPart(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader, Turret turret, PolarVelocityCalculator calculator) {
        Trajectory path = ExtendedTrajectoryUtilities.getDeployedTrajectory("TriangleSecondPart");
        addCommands(
                new ParallelCommandGroup(
                        NewTrajectoryUtilities.generateSwerveControllerCommand(swerve, path),
                                new IntakeRunCommand(intake).withTimeout(0.1)
                ).withTimeout(2),
                new InstantCommand(() -> swerve.moveRobotCentric(0,0,0)),
                new InstantCommand(() -> turret.setAngle(0)),
                new WaitCommand(0.5),
                new ParallelCommandGroup(
                        new AutomatedShootingCommand(shooter, vision, loader, calculator),

                        new IntakeRunCommand(intake, 0)
                ).withTimeout(2),
                new ShooterSpinDownCommand(shooter)
        );
    }
}