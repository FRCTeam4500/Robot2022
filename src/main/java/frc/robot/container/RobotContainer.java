package frc.robot.container;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public interface RobotContainer {

    default Command getAutonomousCommand(){
        return null;
    }

    default void teleopInit(){

    }
}
