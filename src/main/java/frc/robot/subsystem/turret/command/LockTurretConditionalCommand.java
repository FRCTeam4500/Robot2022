package frc.robot.subsystem.turret.command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.turret.Turret;

public class LockTurretConditionalCommand extends CommandBase{
    private Turret turret;
    private Supplier<Boolean> condition;

    public LockTurretConditionalCommand(Turret turret, Supplier<Boolean> condition){
        this.turret = turret;
        this.condition = condition;
    }

    public void execute(){
        if (condition.get()){
            turret.setOutput(0);
            turret.setEnabled(false);
        }
        else{
            turret.setEnabled(true);
        }
    }

    public void end(){
        turret.setEnabled(true);
    }

    public void interrupted(){
        turret.setEnabled(true);
    }
    
}