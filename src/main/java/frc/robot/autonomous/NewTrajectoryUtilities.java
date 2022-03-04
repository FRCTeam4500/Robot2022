package frc.robot.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystem.swerve.SwerveConstants;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;

/**
 * Trajectory utils for built-in wpilib pathfinding
 * Autonomous constants will be here
 */
public class NewTrajectoryUtilities {


    /**
     * Generates a Swerve path following command. This command does not rotate continuously, it targets the end rotation of the path.
     * If you want continuous rotation, use the SwerveControllerCommand constructor which takes rotation input.
     * It's the first one in the file.
     * @param swerve The swerve
     * @param path the path to follow
     */
    public static SequentialCommandGroup generateSwerveControllerCommand(PathFollowingSwerve swerve, Trajectory path){
        return generateSwerveControllerCommand(swerve,path,false);
    }

    /**
     * Generates a Swerve path following command. This command does not rotate continuously, it targets the end rotation of the path.
     * If you want continuous rotation, use the SwerveControllerCommand constructor which takes rotation input.
     * It's the first one in the file.
     * @param swerve The swerve
     * @param path the path to follow
     * @param disableRotation stop the robot from rotating when running the path
     */
    public static SequentialCommandGroup generateSwerveControllerCommand(PathFollowingSwerve swerve, Trajectory path, boolean disableRotation){
        double anglekP = 4;
        if (disableRotation)
            anglekP = 0;

        ProfiledPIDController angleControl = new ProfiledPIDController(anglekP,0,0, new TrapezoidProfile.Constraints(SwerveConstants.MAX_ROTATIONAL_SPEED, SwerveConstants.MAX_ROTATIONAL_ACCELERATION));
        angleControl.enableContinuousInput(-Math.PI/2, Math.PI/2);

        Supplier<Rotation2d> rotation = () -> {return path.getStates().get(path.getStates().size() - 1).poseMeters.getRotation().times(-1);};

    
        return new SwerveControllerCommand(
                path,
                swerve::getCurrentPose,
                swerve.getKinematics(),
                new PIDController(1,0,0),
                new PIDController(1,0,0),
                angleControl,
                rotation,
                swerve::driveByStates,
                swerve
                ).andThen(() -> swerve.moveRobotCentric(0,0,0)); //stop robot when done
    }

}
