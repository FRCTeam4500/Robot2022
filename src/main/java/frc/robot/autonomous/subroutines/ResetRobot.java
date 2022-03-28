package frc.robot.autonomous.subroutines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.lights.Lights;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinDownCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.loader.command.LoaderSetOutputCommand;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.arm.ArmConstants;

public class ResetRobot extends ParallelCommandGroup{
    public ResetRobot(PathFollowingSwerve swerve, Arm arm, Shooter shooter, Loader loader, Intake intake, Lights lights, Rotation2d initialRotation){
        addCommands(
            new InstantCommand(() -> {System.out.println("Resetting robot");}),
            new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE),
            new ShooterSpinDownCommand(shooter),
            new LoaderSetOutputCommand(loader, 0),
            new IntakeRunCommand(intake, 0),
                new InstantCommand(() -> lights.setCurrentRoutine(Lights.Routines.blueorbit)),
            //initial rotation is the rotation from 0 of the first pose of the path, this will be the gyro zero initially
                // the current robot angle is the angle offset from zero, so initial plus current will equal total offset from zero
                //since the physical gyro zero will be wherever the robot was facing when it turned on, the actual zero will be to the left of the physical zero, whether that is negative or positive depends on whether clockwise os negative or positivec
            new InstantCommand(() -> swerve.resetRobotAngle(-(-initialRotation.getRadians() + swerve.getRobotAngle()))) //we might have to play with the signs of these
        );
    }
}