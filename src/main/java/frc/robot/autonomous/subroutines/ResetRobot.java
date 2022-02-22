package frc.robot.autonomous.subroutines;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinDownCommand;
import frc.robot.subsystem.loader.command.LoaderRunCommand;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.arm.ArmConstants;

public class ResetRobot extends ParallelCommandGroup{
    public ResetRobot(Arm arm, Shooter shooter, Loader loader, Intake intake){
        addCommands(
            new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE),
            new ShooterSpinDownCommand(shooter),
            new LoaderRunCommand(loader, 0),
            new IntakeRunCommand(intake, 0)
        );
    }
}