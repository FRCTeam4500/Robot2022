package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;

public class ClimberChainsRunCommand extends InstantCommand{
    private Climber climber;
    private double speed;

    public ClimberChainsRunCommand(Climber climber, double speed){
        this.climber = climber;
        this.speed = speed;
        addRequirements(climber);
    }
    public ClimberChainsRunCommand(Climber climber){
        this(climber, ClimberConstants.CHAINS_RUN_SPEED);
    }

    public void initialize() {
        climber.setOutput(speed);
    }

  
    
}
