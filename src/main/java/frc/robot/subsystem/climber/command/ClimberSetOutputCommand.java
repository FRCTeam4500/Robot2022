package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;

public class ClimberSetOutputCommand extends InstantCommand{
    private Climber climber;
    private double speed;

    public ClimberSetOutputCommand(Climber climber) {
        this.climber = climber;
        this.speed = ClimberConstants.CHAINS_RUN_SPEED;
    }

    public ClimberSetOutputCommand(Climber climber, double output) {
        this.climber = climber;
        this.speed = output;
        addRequirements(climber);
    }

    public void initialize() {
        climber.setOutput(speed);
    }

    
}
