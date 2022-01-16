package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.FirstBall;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.vision.Vision;

public class FirstBallAuto extends SequentialCommandGroup {
    public FirstBallAuto(PathFollowingSwerve swerve, Arm arm, Shooter shooter, Intake intake, Vision vision, Loader loader){
        addCommands(
                new FirstBall(swerve, arm, intake, shooter, vision, loader),
                new ArmSetAngleCommand(arm, 0)
        );
    }
}
