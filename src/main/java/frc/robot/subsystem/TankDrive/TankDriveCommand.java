// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.TankDrive;

import frc.robot.subsystems.TankDrive.TankDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class TankDriveCommand extends CommandBase {
  private TankDrive tankDrive;
  private Joystick joystick;
  private double dead;
  private double sens;

  /** Creates a new TankDriveCommand. */
  public TankDriveCommand(TankDrive tankDrive, Joystick joystick, double deadzone, double sensitivity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.tankDrive = tankDrive;
    this.joystick = joystick;
    dead = deadzone;
    sens = sensitivity;
    addRequirements(tankDrive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = joystick.getX();
    double y = joystick.getY();
    double z = joystick.getZ();
    if (Math.abs(x) < dead) {
      x = 0;
    }

    if (Math.abs(y) < dead) {
      y = 0;
    }

    if (Math.abs(z) < dead) {
      z = 0;
    }

    tankDrive.move(x*sens, y*sens, z*sens);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
