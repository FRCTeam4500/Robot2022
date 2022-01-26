/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.swerve.pathfollowingswerve.command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.KinematicSwerve;
import frc.robot.subsystem.swerve.SwerveControl;
import frc.robot.utility.ControllerInfo;

public class SwerveDefaultCommand extends CommandBase {
    private KinematicSwerve swerve;
    private Joystick control;
    private ControllerInfo info;
    /**
     */
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = control.getX();
        double y = control.getY();
        double w = control.getZ();
        if (Math.abs(x) < info.xDeadzone){
            x = 0;
        }
        if (Math.abs(y) < info.yDeadzone){
            y = 0;
        }
        if (Math.abs(w) < info.zDeadzone){
            w = 0;
        }


        swerve.moveFieldCentric(-x * info.xSensitivity, y * info.ySensitivity, w * info.zSensitivity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.moveFieldCentric(0,0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public SwerveDefaultCommand(KinematicSwerve swerve, Joystick control, ControllerInfo info) {
        this.swerve = swerve;
        this.control = control;
        this.info = info;
        addRequirements(swerve);
    }

}

