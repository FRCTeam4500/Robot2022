package frc.robot.subsystem.intake.command;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeControl;

/**
 * Command which sets an Intake according to the angle value stored in an IntakeControl
 *
 *
 */

public class IntakeDefaultCommand extends CommandBase {

    private Intake intake;
    private IntakeControl control;
    public IntakeDefaultCommand (Intake intake, IntakeControl control){
        this.intake = intake;
        this.control = control;
    }

    public void execute(){
        intake.setSpeed(control.speed);
    }
}
