// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.swerve.pathfollowingswerve.command;

import java.util.function.DoubleFunction;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.utility.ExtendedMath;

public class FollowStatesCommand extends FollowTrajectoryCommand {

    protected double internalTime = 0.0;
    protected DoubleFunction<Double> timeTransformer;
    public FollowStatesCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                               HolonomicDriveController controller, DoubleFunction<Double> timeTransformer) {
        super(swerve, trajectory, controller);
        this.timeTransformer = timeTransformer;
    }
    @Override
    public void initialize() {
        internalTime = 0.0;
        currentTranslation = trajectory.getInitialPose().getTranslation();
    }
    @Override
    public void execute() {
        var distance = ExtendedMath.distance(swerve.getCurrentPose().getTranslation(), currentTranslation);
        internalTime += (timeTransformer != null)? timeTransformer.apply(distance) : 1/distance;
        applyState(trajectory.sample(internalTime));
    }
    @Override
    public boolean isFinished() {
        return internalTime >= trajectory.getTotalTimeSeconds() && controller.atReference();
    }
}
