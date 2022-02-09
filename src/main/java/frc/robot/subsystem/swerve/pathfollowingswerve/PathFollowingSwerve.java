package frc.robot.subsystem.swerve.pathfollowingswerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.subsystem.swerve.Swerve;

public interface PathFollowingSwerve extends Swerve, Sendable {
    public Pose2d getCurrentPose();
    public void resetPose(Translation2d translation);
}
