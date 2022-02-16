package frc.robot.subsystem.shooter.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.util.ShooterControl;

import java.util.function.DoubleSupplier;

public class ShooterContinuousRunCommand extends CommandBase {
    private Shooter shooter;
    private DoubleSupplier speed;

    public ShooterContinuousRunCommand(Shooter shooter, DoubleSupplier speed){
        this.shooter = shooter;
        this.speed = speed;
        shooter.setSpeed(speed.getAsDouble());
    }

    public void execute(){
        
    }

    public void end(){
        shooter.setSpeed(0);
    }

}
