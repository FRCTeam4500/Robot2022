package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterConstants;

public class ShooterSetAngleCommand extends CommandBase {
    private Shooter shooter;
    private double targetAngle;
    private double angleThreshold;

    public ShooterSetAngleCommand(Shooter shooter, double angle, double angleThreshold){
        //There is no addRequirements because the shooter's angle is only set once.
        //Having an addRequirements would break stuff bc this command usually runs in parallel with the spin up command
        this.shooter = shooter;
        this.targetAngle = angle;
        this.angleThreshold = angleThreshold;
    }

    public ShooterSetAngleCommand(Shooter shooter, double angle){
        this(shooter, angle, ShooterConstants.angleThreshold);
    }

    public void initialize(){
        shooter.setAngle(targetAngle);
    }

    public boolean isFinished(){
        return Math.abs(targetAngle - shooter.getAngle()) <= angleThreshold;
    }
}
