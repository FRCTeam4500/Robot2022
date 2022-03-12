package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.subroutines.FirstBall;
import frc.robot.autonomous.subroutines.ResetRobot;
import frc.robot.autonomous.subroutines.SecondBallDirect;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.lights.Lights;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.PolarVelocityCalculator;

public class SecondBallAuto extends SequentialCommandGroup {
    public SecondBallAuto(PathFollowingSwerve swerve, Arm arm, Shooter shooter, Intake intake, Vision vision, Loader loader, Turret turret, Lights lights, PolarVelocityCalculator calculator){
        addCommands(
                new SecondBallDirect(swerve, arm, intake, shooter, vision, loader, turret, lights, calculator),
                new ResetRobot(arm, shooter, loader, intake, lights)
        );
    }
}
