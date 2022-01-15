package frc.robot.subsystem.swerve.pathfollowingswerve;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystem.swerve.Swerve;

public interface PathFollowingSwerve extends Swerve {
    public Pose2d getCurrentPose();
    public void resetPose(Translation2d translation);
}
