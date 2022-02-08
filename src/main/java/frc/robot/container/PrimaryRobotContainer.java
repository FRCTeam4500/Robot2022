package frc.robot.container;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.AlignWithTargetCommand;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.swerve.command.SwerveDefaultCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ControllerInfo;

import javax.naming.ldap.Control;

public class PrimaryRobotContainer implements RobotContainer{


    private Swerve swerve;
    private Vision vision;

    private Joystick driveStick = new Joystick(1);
    private JoystickButton switchDriveMode = new JoystickButton(driveStick, 1);
    private JoystickButton b2 = new JoystickButton(driveStick, 2);

    ControllerInfo controllerInfo = new ControllerInfo();


    public PrimaryRobotContainer(){

        controllerInfo.xDeadzone = 0.2;

        SmartDashboard.putNumber("align kp", 1);


        b2.whenPressed(() -> {swerve.moveRobotCentric(0,0,1);});

    }

    void configureSwerve(){
        SwerveDefaultCommand swerveCommand = new SwerveDefaultCommand(swerve, driveStick, controllerInfo);
        switchDriveMode.whenPressed(() -> {swerveCommand.isRobotCentric = true;});
        switchDriveMode.whenReleased(() -> {swerveCommand.isRobotCentric = false;});
        swerve.setDefaultCommand(swerveCommand); 
    }



}
