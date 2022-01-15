package frc.robot.subsystem.intake.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.intake.Intake;

/**
 * Command for manually controlling an Intake,
 * takes over control of the Intake from the defaultcommand, so you don't have to pass an IntakeControl to a CommandGroup
 */

public class IntakeSetSpeedCommand extends CommandBase {
    private Intake intake;
    private double speed;
    public IntakeSetSpeedCommand (Intake intake, double speed){
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    public void initialize(){
        intake.setSpeed(speed);
    }
}
