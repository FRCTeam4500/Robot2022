// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.swerve.pathfollowingswerve.command;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowTrajectoryCommand;
import frc.robot.utility.ExtendedMath;

/**
 * Further improved version of FollowTrajectoryCommand, uses a dot product and ghost robot to follow paths.
 *
 */


public class FollowDottedTrajectoryCommand extends FollowTrajectoryCommand {

    protected double internalTime = 0.0, timeStep = 0.02, threshold = 0.1;

    public FollowDottedTrajectoryCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                                         HolonomicDriveController controller) {
        super(swerve, trajectory, controller);
    }

    @Override
    public void initialize() {
        internalTime = 0.0;
    }
    @Override
    public void execute() {

        while(internalTime < trajectory.getTotalTimeSeconds() && signedDistance() < threshold){
            internalTime += timeStep;
        }
        applyState(trajectory.sample(internalTime));
    }
    double signedDistance(){
        Trajectory.State sample = trajectory.sample(internalTime);
        Pose2d currPose = swerve.getCurrentPose();
        Translation2d displacement = sample.poseMeters.getTranslation().minus(currPose.getTranslation());
        Translation2d direction = new Translation2d(sample.poseMeters.getRotation().getCos(), sample.poseMeters.getRotation().getSin());
        return Math.signum(ExtendedMath.dot(displacement, direction))*displacement.getNorm();
    }
    @Override
    public boolean isFinished() {
        return internalTime >= trajectory.getTotalTimeSeconds() && currentTranslation.getDistance(swerve.getCurrentPose().getTranslation()) <= threshold;
    }
    public FollowDottedTrajectoryCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                                                         HolonomicDriveController controller, double threshold, double timeStep) {
        super(swerve, trajectory, controller);
        this.timeStep = timeStep;
        this.threshold = threshold;
    }
    public FollowDottedTrajectoryCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                                                         HolonomicDriveController controller, double threshold, double timeStep, boolean rotation, double rotationOffset) {
        super(swerve, trajectory, controller);
        this.timeStep = timeStep;
        this.threshold = threshold;
        setRotation(rotation);
        setDesiredRotationOffset(rotationOffset);
    }
    public FollowDottedTrajectoryCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                                                         HolonomicDriveController controller, double threshold) {
        super(swerve, trajectory, controller);
        this.threshold = threshold;
    }
    public double getTimeStep() {
        return timeStep;
    }
    public void setTimeStep(double timeStep) {
        this.timeStep = timeStep;
    }
    public double getThreshold() {
        return threshold;
    }
    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Timestep", this::getTimeStep, this::setTimeStep);
        builder.addDoubleProperty("Threshold", this::getThreshold, this::setThreshold);
    }

    public void end(){
        swerve.moveRobotCentric(0,0,0);
    }

    public void interrupted(){
        end();
    }
}

