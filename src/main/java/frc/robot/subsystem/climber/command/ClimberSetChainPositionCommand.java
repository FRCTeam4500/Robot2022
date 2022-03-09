package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.climber.Climber;

public class ClimberSetChainPositionCommand extends InstantCommand {
    private Climber climber;
    private int position;

    public ClimberSetChainPositionCommand(Climber climber, int position){
        this.climber = climber;
        this.position = position;
    }

    public void initialize(){
        climber.setChainPosition(position);
    }
}
