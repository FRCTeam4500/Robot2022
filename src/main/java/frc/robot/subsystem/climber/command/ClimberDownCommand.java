package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;

public class ClimberDownCommand extends CommandBase {
    private Climber climber;
    private double angle;

    public ClimberDownCommand(Climber climber, double angle) { //TODO: if we are using output
        this.climber = climber;
        this.angle = angle;
    }

    public ClimberDownCommand(Climber climber) {
        this.climber = climber;
        this.angle = ClimberConstants.CLIMBER_DOWN_ANGLE;
    }

    public void initialize() {
        climber.setAngle(angle);
    }

    public void end() {
        climber.setAngle(0);
    }
}
