package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;

public class ClimberRunCommand extends CommandBase{
    private Climber climber;
    private double speed;

    public ClimberRunCommand(Climber climber) {
        this.climber = climber;
        this.speed = ClimberConstants.MAX_OUTPUT;
    }

    public ClimberRunCommand(Climber climber, double output) {
        this.climber = climber;
        this.speed = output;
        addRequirements(climber);
    }

    public void initialize() {
        climber.setOutput(speed);
    }

    public void end() {climber.setOutput(0);}
    
}
