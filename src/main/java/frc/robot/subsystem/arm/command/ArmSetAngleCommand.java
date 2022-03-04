package frc.robot.subsystem.arm.command;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.arm.Arm;

/**
 * Command for manually controlling an Arm,
 */

public class ArmSetAngleCommand extends InstantCommand {
    private Arm arm;
    private double angle;

    public ArmSetAngleCommand(Arm arm, double angle){
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    public void initialize(){
        arm.setAngle(angle);
    }
}
