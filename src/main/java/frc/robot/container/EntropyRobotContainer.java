package frc.robot.container;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.routinesDriveOnly.FirstBallAuto;
import frc.robot.autonomous.routinesDriveOnly.TriangleAuto;
import frc.robot.autonomous.routinesDriveOnly.WAuto;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.SwerveDefaultCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.factory.EntropySwerveFactory;

public class EntropyRobotContainer implements RobotContainer{


    private OdometricSwerve swerve;

    private Joystick driveStick = new Joystick(0);

    private SendableChooser<Command> commandChooser = new SendableChooser();

    private void configureSwerve(){
        swerve = EntropySwerveFactory.makeSwerve();
        swerve.setDefaultCommand(new SwerveDefaultCommand(swerve, driveStick));
    }

    private void configureAutonomous(){
        Command firstBallAuto = new FirstBallAuto(swerve);
        Command triangleAuto = new TriangleAuto(swerve);
        Command wAuto = new WAuto(swerve);

        commandChooser.addOption("First Ball Auto", firstBallAuto);
        commandChooser.addOption("Triangle Auto", triangleAuto);
        commandChooser.addOption("W Auto", wAuto);
    }

    public Command getAutonomous(){
        return commandChooser.getSelected();
    }


}
