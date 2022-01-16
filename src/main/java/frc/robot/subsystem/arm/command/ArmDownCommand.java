package frc.robot.subsystem.arm.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;

/**
 * Command which puts down an arm until the command ends
 * Use ArmSetAngleCommand if you want the arm to stay down after the command ends.
 * Use the default command with an ArmControl if you need to continuously vary the angle
 * Defaults to value set in ArmConstants if no angle is given
 */

public class ArmDownCommand extends CommandBase {
    private Arm arm;
    private double angle;

    public ArmDownCommand(Arm arm, double angle) {
        this.arm = arm;
        this.angle = angle;
        addRequirements(arm);
    }

    public ArmDownCommand(Arm arm) {
        this(arm, ArmConstants.armDownAngle);
    }

    public void initialize() {
        arm.setAngle(angle);
    }

    public void end() {
        arm.setAngle(0);
    }
}
