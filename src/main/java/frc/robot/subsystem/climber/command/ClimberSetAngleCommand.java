package frc.robot.subsystem.climber.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.climber.Climber;

public class ClimberSetAngleCommand extends CommandBase {
    private Climber climber;
    private double angle;

    public ClimberSetAngleCommand(Climber climber, double angle) {
        this.climber = climber;
        this.angle = angle;
        addRequirements(climber);
    }

    public void initialize() {
        climber.setAngle(angle);
    }
}
