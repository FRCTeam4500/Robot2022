package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;

public class ClimberUpCommand extends CommandBase {
    private Climber climber;
    private double angle;

    public ClimberUpCommand(Climber climber, double angle) { //TODO: If this needs output instead just change this lol
        this.climber = climber;
        this.angle = angle;
    }

    public ClimberUpCommand(Climber climber) {
        this.climber = climber;
        this.angle = ClimberConstants.CLIMBER_UP_ANGLE;
    }

    public void initialize() {
        climber.setAngle(angle);
    }

    public void end() {
        climber.setAngle(0);
    }
}
