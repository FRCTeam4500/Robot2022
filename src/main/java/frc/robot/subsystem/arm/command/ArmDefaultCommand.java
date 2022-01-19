package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmControl;

/**
 * Command which controls an Arm according to an ArmControl
 */

public class ArmDefaultCommand extends CommandBase {
    public Arm arm;
    public ArmControl control;

    public ArmDefaultCommand(Arm arm, ArmControl control){
        this.arm = arm;
        this.control = control;
        addRequirements(arm);
    }

    public void execute(){
        arm.setAngle(control.angle);
    }
}
