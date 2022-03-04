package frc.robot.subsystem.intake.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;

public class IntakeSetOutputCommand extends InstantCommand {
    private Intake intake;
    private double output;


    public IntakeSetOutputCommand(Intake intake, double output){
        this.intake = intake;
        this.output = output;
    }

    public IntakeSetOutputCommand(Intake intake){
        this(intake, IntakeConstants.intakeRunSpeed);
    }

    public void initialize(){
        intake.setOutput(output);
    }
}
