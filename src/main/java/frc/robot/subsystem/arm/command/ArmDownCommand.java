package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;

/**
 * Command which puts down the arm
 * Use ArmSetAngleCommand if you need the arm to stay down
 */

public class ArmDownCommand extends CommandBase {
    private Arm arm;
    private double angle;

    public ArmDownCommand(Arm arm, double angle){
        this.arm = arm;
        this.angle = angle;
    }

    public ArmDownCommand(Arm arm){
        this(arm, ArmConstants.armDownAngle);
    }

    public void initialize(){
        arm.setAngle(angle);
    }

    public void end(){
        arm.setAngle(0);
    }

}
