// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.swerve.pathfollowingswerve.command;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

/**
 * Improved path following command.
 * Does not rotate the robot be default, call setRotation to change
 */

public class FollowTrajectoryWithEndRotationOffsetCommand extends CommandBase {

    protected PathFollowingSwerve swerve;
    protected Trajectory trajectory;
    protected Timer timer = new Timer();
    protected HolonomicDriveController controller;
    protected double[] currentState = new double[]{0,0};
    protected Translation2d currentTranslation;
    protected boolean rotation = false;
    protected Rotation2d desiredRotationOffset;

    private Rotation2d targetOffset;
    private Rotation2d swerveReferenceAngle = new Rotation2d();
    private PIDController rotationController = new PIDController(-0.7,0,0);;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        currentTranslation = trajectory.getInitialPose().getTranslation();
        swerveReferenceAngle = new Rotation2d(swerve.getRobotAngle());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Trajectory.State state = trajectory.sample(timer.get());
        applyState(state);
    }

    protected void applyState(Trajectory.State state) {
        currentTranslation = state.poseMeters.getTranslation();
        currentState[0] = state.poseMeters.getX();
        currentState[1] = state.poseMeters.getY();
        System.out.println(swerveReferenceAngle);
        Rotation2d rotationOutput = new Rotation2d(rotationController.calculate(swerve.getCurrentPose().getRotation().getRadians(),
         swerveReferenceAngle.plus(
             targetOffset).getRadians())); //Calculates target rotational speed by trying to match the current rotation to the target offset plus initial rotation

        ChassisSpeeds output = controller.calculate(swerve.getCurrentPose(), state, rotationOutput);
        swerve.moveFieldCentric(output.vxMetersPerSecond, output.vyMetersPerSecond, -output.omegaRadiansPerSecond);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.get() >= trajectory.getTotalTimeSeconds() && controller.atReference();
    }

    public FollowTrajectoryWithEndRotationOffsetCommand(PathFollowingSwerve swerve, Trajectory trajectory,
                                                        HolonomicDriveController controller, Rotation2d targetOffset) {
        this.swerve = swerve;
        this.trajectory = trajectory;
        this.controller = controller;
        this.targetOffset = targetOffset;
        addRequirements(swerve);
    }



    public Trajectory getTrajectory() {
        return trajectory;
    }

    public void setTrajectory(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public HolonomicDriveController getController() {
        return controller;
    }

    public void setController(HolonomicDriveController controller) {
        this.controller = controller;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleArrayProperty("Current State", () -> currentState, null);
        builder.addDoubleProperty("Trajectory Time", () -> trajectory.getTotalTimeSeconds(), null);
    }
    public void setRotation(boolean bool){
        rotation = bool;
    }
    public void setDesiredRotationOffset(double offset){
        desiredRotationOffset = new Rotation2d(offset);
    }
    public void end(){
        swerve.moveRobotCentric(0,0,0);
    }

    public void interrupted(){
        end();
    }
}


