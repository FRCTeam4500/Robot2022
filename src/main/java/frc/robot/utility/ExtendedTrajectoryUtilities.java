/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utility;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowDottedTrajectoryCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowStatesCommand;
import frc.robot.subsystem.swerve.pathfollowingswerve.command.FollowTrajectoryCommand;



/**
 * Utilities for running autonomous paths
 * A bunch of the included methods will not be used, for the most part we will only be using getDeployedTrajectory()
 */
public class ExtendedTrajectoryUtilities {
    private static Trajectory getDeployedTrajectoryExcept(String trajectoryName) throws IOException {

        var trajectoryJSON = "paths/"+trajectoryName+".wpilib.json";
        //Stolen pretty much from the example code
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        return trajectory;
    }
    public static Trajectory getDeployedTrajectory(String trajectoryName){
        try{
            return getDeployedTrajectoryExcept(trajectoryName);
        }catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
            return new Trajectory(null);
        }
    }
    public static Trajectory regenerateTrajectory(Trajectory trajectory, TrajectoryConfig config){
        return TrajectoryGenerator.generateTrajectory(Arrays.asList(trajectory.getStates().stream().map(s -> s.poseMeters).toArray(Pose2d[]::new)), config);
    }

    /**
     * Creates a trajectory command and adds it to Shuffleboard
     * @param swerve The swerve that runs the path
     * @param tabName the tab on which to place the command and info
     * @param trajectoryName the name of the trajectory to run
     * @return An InstantCommand wrapper for the FollowStatesCommand
     */
    public static CommandBase addTrajectoryWithShuffleboard(PathFollowingSwerve swerve, String tabName, String trajectoryName){ //Adds a trajectory with a shuffleboard tab for controlling said trajectory

        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        NetworkTableEntry timeAdvanceEntry = tab.addPersistent("Time Advance", 0.02).getEntry();
        NetworkTableEntry distanceMultiplerEntry = tab.addPersistent("Distance Multiplier", 3).getEntry();

        Trajectory trajectory = getDeployedTrajectory(trajectoryName);
        FollowStatesCommand followCommand = new FollowStatesCommand(swerve, trajectory, createBasicController(1, 1, 1, 4, 1), distance -> {
            return Math.min(timeAdvanceEntry.getDouble(0.02), 1/(distance * distanceMultiplerEntry.getDouble(3)));
        });

        createTrajectoryRegenerationLayout(swerve, tab, trajectory, followCommand);

        createControllerRegenerationLayout(swerve, tab, followCommand);

        return createAndAddFinalFollowCommand(swerve, tab, trajectory, followCommand);

    }

    /**
     * Creates a dotted trajectory command and adds it to shuffleboard
     * @param swerve The PathFollowingSwerve to run the command
     * @param tabName The tab on which to add the command
     * @param trajectoryName The name of the trajectory
     * @return An InstantCommand wrapper for the DottedTrajectoryCommand
     */

    public static CommandBase addDottedTrajectoryWithShuffleboard(PathFollowingSwerve swerve, String tabName, String trajectoryName){
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        Trajectory trajectory = getDeployedTrajectory(trajectoryName);
        FollowDottedTrajectoryCommand followCommand = new FollowDottedTrajectoryCommand(swerve, trajectory, createBasicController(1, 1, 1, 4, 1), 0.01, 0.02);
        createTrajectoryRegenerationLayout(swerve, tab, trajectory, followCommand);

        createControllerRegenerationLayout(swerve, tab, followCommand);

        return createAndAddFinalFollowCommand(swerve, tab, trajectory, followCommand);
    }


    public static CommandBase addDottedTrajectoryWithShuffleboard(PathFollowingSwerve swerve, String tabName, String trajectoryName, boolean rotation, double rotationOffset){
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        Trajectory trajectory = getDeployedTrajectory(trajectoryName);
        FollowDottedTrajectoryCommand followCommand = new FollowDottedTrajectoryCommand(swerve, trajectory, createBasicController(1, 1, 1, 4, 1), 0.01, 0.02, rotation, rotationOffset);
        createTrajectoryRegenerationLayout(swerve, tab, trajectory, followCommand);

        createControllerRegenerationLayout(swerve, tab, followCommand);

        return createAndAddFinalFollowCommand(swerve, tab, trajectory, followCommand);
    }
    private static CommandBase createAndAddFinalFollowCommand(PathFollowingSwerve swerve, ShuffleboardTab tab, Trajectory trajectory, FollowTrajectoryCommand  followCommand) {
        SequentialCommandGroup realFollow = new InstantCommand(() -> swerve.resetPose(trajectory.getInitialPose().getTranslation()), swerve).andThen(followCommand);
        tab.add("Follow Command Diagnostics", followCommand); //path command
        tab.add("Run Follow Command", realFollow); //Command which runs the path command
        return realFollow;
    }
    public static void createControllerRegenerationLayout(PathFollowingSwerve swerve, ShuffleboardTab tab, FollowTrajectoryCommand followCommand) {
        ShuffleboardLayout controllerLayout = tab.getLayout("Controller Regeneration", BuiltInLayouts.kList);
        NetworkTableEntry kPxEntry = controllerLayout.add("kPx", 1).getEntry();
        NetworkTableEntry kPyEntry = controllerLayout.add("kPy", 1).getEntry();
        NetworkTableEntry kPwEntry = controllerLayout.add("kPw", 1).getEntry();
        NetworkTableEntry maxRotSpeedEntry = controllerLayout.add("Max Rotational Speed", 4).getEntry();
        NetworkTableEntry maxRotAccelerationEntry = controllerLayout.add("Max Rotational Acceleration", 1).getEntry();
        controllerLayout.add("Regenerate Controller", new InstantCommand(() -> {
            followCommand.setController(createBasicController(
                    kPxEntry.getDouble(1),
                    kPyEntry.getDouble(1),
                    kPwEntry.getDouble(1),
                    maxRotSpeedEntry.getDouble(4),
                    maxRotAccelerationEntry.getDouble(1)));
        }, swerve));
    }
    public static void createTrajectoryRegenerationLayout(PathFollowingSwerve swerve, ShuffleboardTab tab, Trajectory trajectory, FollowTrajectoryCommand followCommand) {
        ShuffleboardLayout trajectoryLayout = tab.getLayout("Trajectory Regeneration", BuiltInLayouts.kList);
        NetworkTableEntry maxVelocityEntry = trajectoryLayout.add("Max Velocity Meters", 2.4).getEntry();
        NetworkTableEntry maxAccelerationEntry = trajectoryLayout.add("Max Acceleration Meters", 0.5).getEntry();
        NetworkTableEntry maxCentripetalAccelerationEntry = trajectoryLayout.add("Max Centripetal Acceleration Meters", 0.5).getEntry();
        trajectoryLayout.add("Regenerate Trajectory", new InstantCommand(() -> {

            TrajectoryConfig config =  new TrajectoryConfig(
                    maxVelocityEntry.getDouble(2.4),
                    maxAccelerationEntry.getDouble(0.5));
            config.addConstraint(new CentripetalAccelerationConstraint(maxCentripetalAccelerationEntry.getDouble(0.5)));
            followCommand.setTrajectory(
                    ExtendedTrajectoryUtilities.regenerateTrajectory(
                            trajectory,
                            config));
        }, swerve));
    }
    public static HolonomicDriveController createBasicController(double kPx, double kPy, double kPw, double maxRotationalSpeed, double maxRotationlAcceleration){
        return new HolonomicDriveController(new PIDController(kPx, 0, 0), new PIDController(kPy, 0, 0), new ProfiledPIDController(kPw, 0, 0, new TrapezoidProfile.Constraints(maxRotationalSpeed, maxRotationlAcceleration) ));
    }
}
