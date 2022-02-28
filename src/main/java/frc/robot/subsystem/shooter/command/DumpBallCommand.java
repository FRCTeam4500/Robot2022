

package frc.robot.subsystem.shooter.command;

import frc.robot.subsystem.turret.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.loader.Loader;


public class DumpBallCommand extends CommandBase{

    private Turret turret;
    private Shooter shooter;
    private Vision vision;
    private Loader loader;
    private double targetAngle;

    public DumpBallCommand(Turret turret, Shooter shooter, Vision vision, Loader loader){
        this.turret = turret;
        this.shooter = shooter;
        this.vision = vision;
        this.loader = loader;
        addRequirements(turret, shooter);
    }

    public void initialize(){
        if (turret.getAngle() > 0)
            targetAngle = -Math.PI/3;
        if (turret.getAngle() <= 0)
            targetAngle = Math.PI/3;
    }


    public void execute(){
        turret.setAngle(targetAngle);
        shooter.setSpeed(10000);
        if (shooter.atSpeed()){
            loader.setOutput(1);
        }
    }

    public void end(){
        loader.setOutput(0);
        shooter.setSpeed(0);
    }

    public void interrupted(){
        end();
    }

}