package frc.robot.subsystem.swerve.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.ControllerInfo;

public class SwerveDefaultCommand extends CommandBase {
    private Swerve swerve;
    private Joystick joystick;
    private ControllerInfo info;
    public boolean isRobotCentric;

    public SwerveDefaultCommand(Swerve swerve, Joystick joystick, ControllerInfo info){
        this.swerve = swerve;
        this.joystick = joystick;
        this.info = info;
        addRequirements(swerve);
    }

    public void execute(){
        double xSpeed = joystick.getX();
        double ySpeed = joystick.getY();
        double zSpeed = joystick.getZ();
        if (Math.abs(xSpeed) < info.xDeadzone)
            xSpeed = 0;
        if (Math.abs(ySpeed) < info.yDeadzone)
            ySpeed = 0;
        if (Math.abs(zSpeed) < info.zDeadzone)
            zSpeed = 0;
        if (isRobotCentric){
            swerve.moveRobotCentric(
                    xSpeed * info.xSensitivity,
                    ySpeed * info.ySensitivity,
                    zSpeed * info.zSensitivity
            );
            return;
        }
        swerve.moveFieldCentric(
                xSpeed * info.xSensitivity,
                ySpeed * info.ySensitivity,
                zSpeed * info.zSensitivity
        );
    }
}
