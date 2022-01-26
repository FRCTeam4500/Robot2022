package frc.robot.container;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autonomous.routinesDriveOnly.FirstBallAuto;
import frc.robot.autonomous.routinesDriveOnly.TriangleAuto;
import frc.robot.autonomous.routinesDriveOnly.WAuto;
import frc.robot.subsystem.swerve.pathfollowingswerve.OdometricSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.SwerveDefaultCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.factory.EntropySwerveFactory;
import frc.robot.utility.ControllerInfo;

public class EntropyRobotContainer implements RobotContainer{


    private OdometricSwerve swerve;

    private Joystick driveStick = new Joystick(0);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 5);

    private SendableChooser<Command> commandChooser = new SendableChooser();

    public EntropyRobotContainer(){
        configureSwerve();
        //configureAutonomous();
    }

    private void configureSwerve(){
        ControllerInfo info = new ControllerInfo();
        info.xDeadzone = 0.2;
        info.yDeadzone = 0.2;
        info.zDeadzone = 0.2;
        info.xSensitivity = -4;
        info.ySensitivity = 4;
        info.zSensitivity = -4;
        swerve = EntropySwerveFactory.makeSwerve();
        swerve.setDefaultCommand(new SwerveDefaultCommand(swerve, driveStick, info));
        Shuffleboard.getTab("swerve").add(swerve);
        SmartDashboard.putData(swerve);

        resetGyro.whenPressed(new InstantCommand(() -> {swerve.resetPose();}));

        SmartDashboard.putNumber("amogus", 1919);
        Shuffleboard.getTab("Driver Controls").add("aaa", "aaa");
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
