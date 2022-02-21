package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;

public class ArmDownCommand extends CommandBase {

    private Arm arm;
    private double angle;

    public ArmDownCommand(Arm arm, double angle){
        this.arm = arm;
        this.angle = angle;
    }
    public ArmDownCommand(Arm arm){
        this(arm, ArmConstants.ARM_DOWN_ANGLE);
    }

    public void initialize(){
        arm.setAngle(angle);
    }

    public void end(){
        arm.setAngle(ArmConstants.ARM_UP_ANGLE);
    }
}
