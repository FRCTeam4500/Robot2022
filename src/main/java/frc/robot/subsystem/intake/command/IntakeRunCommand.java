package frc.robot.subsystem.intake.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;

/**
 * Runs an intake until the command ends
 * takes over control of the Intake from the defaultcommand, so you don't have to pass an IntakeControl to a CommandGroup
 * We don't need to provide an intake set speed command because the intake will never be left on.
 * Speed defaults to the speed set in IntakeConstants if not given to constructor
 */

public class IntakeRunCommand extends CommandBase {
    private Intake intake;
    private double speed;
    public IntakeRunCommand(Intake intake, double speed){
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }
    public IntakeRunCommand(Intake intake){
        this(intake, IntakeConstants.intakeRunSpeed);
    }

    public void initialize(){
        intake.setSpeed(speed);
    }

    public void end() {
        intake.setSpeed(0);
    }
}
