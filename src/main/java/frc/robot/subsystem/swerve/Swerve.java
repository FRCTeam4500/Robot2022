package frc.robot.subsystem.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Swerve extends Subsystem {
    public void moveRobotCentric( double xSpeed, double ySpeed, double wSpeed );
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds);
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation);
    public void moveAngleCentric(double xSpeed, double ySpeed, double wSpeed, Rotation2d robotAngle);
    public void moveFieldCentric(ChassisSpeeds chassisSpeeds);
    public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed);
    public void moveFieldCentric(double xSpeed, double ySpeed, double wSpeed, Translation2d centerOfRotation);
    public void resetGyro();
}
