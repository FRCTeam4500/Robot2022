package frc.robot.container;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.AlignWithTargetCommand;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ControllerInfo;

import javax.naming.ldap.Control;

public class PrimaryRobotContainer implements RobotContainer{


    private Swerve swerve;
    private Vision vision;

    private Joystick driveStick = new Joystick(1);
    private JoystickButton b1 = new JoystickButton(driveStick, 1);

    ControllerInfo controllerInfo = new ControllerInfo();


    public PrimaryRobotContainer(){

        controllerInfo.xDeadzone = 0.2;

        SmartDashboard.putNumber("align kp", 1);

        b1.whenPressed(new AlignWithTargetCommand(SmartDashboard.getNumber("align kp", 1),0,0, vision, swerve));
    }




}
