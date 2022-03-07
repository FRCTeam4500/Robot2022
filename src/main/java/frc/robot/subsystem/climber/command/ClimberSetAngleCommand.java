package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.climber.Climber;

public class ClimberSetAngleCommand extends CommandBase {
    private Climber climber;
    private int position;

    public ClimberSetAngleCommand(Climber climber, int position) {
        this.climber = climber;
        this.position = position;
        addRequirements(climber);
    }

    public void initialize() {
        climber.setPosition(position);
    }
}
