package frc.robot.subsystem.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Swerve extends Subsystem, Sendable {
    public void moveRobotCentric( double ySpeed, double xSpeed, double wSpeed );
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds);
    public void moveRobotCentric(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation);
    public void moveAngleCentric(double ySpeed, double xSpeed, double wSpeed, Rotation2d robotAngle);
    public void moveFieldCentric(ChassisSpeeds chassisSpeeds);
    public void moveFieldCentric(double ySpeed, double xSpeed, double wSpeed, Translation2d centerOfRotation);
    public void resetGyro();

    void moveFieldCentric(double vyMetersPerSecond, double vxMetersPerSecond, double omegaRadiansPerSecond);
}
