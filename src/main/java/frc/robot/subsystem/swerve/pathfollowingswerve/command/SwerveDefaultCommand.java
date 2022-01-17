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

public class SwerveDefaultCommand extends CommandBase {
    private KinematicSwerve swerve;
    private Joystick control;
    /**
     */
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.moveFieldCentric(control.getX(), control.getY(), control.getZ());
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

    public SwerveDefaultCommand(KinematicSwerve swerve, Joystick control) {
        this.swerve = swerve;
        this.control = control;
        addRequirements(swerve);
    }

}

