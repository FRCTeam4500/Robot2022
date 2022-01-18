package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.Shooter;

/**
 * Spins up the shooter.
 * Finishes when the shooter is at the specified speed.
 */

public class SpinUpCommand extends CommandBase {
    private Shooter shooter;
    private double speed; //target speed for the shooter
    private double speedThreshold; //maximum allowable difference between target and shooter speed for the shooter to be considered "at speed"

    public SpinUpCommand(Shooter shooter, double speed, double speedThreshold){
        this.shooter = shooter;
        this.speed = speed;
        this.speedThreshold = speedThreshold;
        addRequirements(shooter);
    }

    public void initialize(){
        shooter.setSpeed(speed);
    }

    public boolean isFinished(){
        return Math.abs(shooter.getSpeed() - speed) <= speedThreshold;
    }
}
