package frc.robot.subsystem.swerve;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Swerve extends Subsystem {
    public void moveRobotCentric( double xSpeed, double ySpeed, double zSpeed );
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds);
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation);
    public void moveAngleCentric(ChassisSpeeds chassisSpeeds);
    public void moveAngleCentric(double xSpeeds, double ySpeeds, double zSpeeds);
    public void moveFieldCentric(ChassisSpeeds chassisSpeeds);
    public void moveFieldCentric(double xSpeeds, double ySpeeds, double zSpeeds);
    public void resetGyro();
}
