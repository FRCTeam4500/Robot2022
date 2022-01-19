package frc.robot.subsystem.arm.command;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.arm.Arm;

/**
 * Command for manually controlling an Arm,
 * takes over control of the Arm from the defaultcommand, so you don't have to pass an ArnControl to a CommandGroup
 */

public class ArmSetAngleCommand extends CommandBase {
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
