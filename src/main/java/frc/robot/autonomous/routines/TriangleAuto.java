package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.FirstBall;
import frc.robot.autonomous.subroutines.ResetRobot;
import frc.robot.autonomous.subroutines.TriangleSecondPart;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.vision.Vision;

public class TriangleAuto extends SequentialCommandGroup {
    public TriangleAuto(PathFollowingSwerve swerve, Arm arm, Intake intake, Shooter shooter, Vision vision, Loader loader) {
        addCommands(
                new FirstBall(swerve, arm, intake, shooter, vision, loader),
                new TriangleSecondPart(swerve, arm, intake, shooter, vision, loader),
                new ResetRobot(arm, shooter, loader, intake)
        );
    }
}
