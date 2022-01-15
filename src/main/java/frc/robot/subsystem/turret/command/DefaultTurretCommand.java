package frc.robot.subsystem.turret.command;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;

/**
 * Default command for turret
 * Automatically aims it at target
 */

public class DefaultTurretCommand extends CommandBase {
    private Turret turret;
    private Vision vision;
    private PIDController controller;

    public DefaultTurretCommand(Turret turret, Vision vision){
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
    }

    public void periodic(){
        //TODO: aim the turret with the pid controller
        //not doing this yet so i can explain it in person
    }
}
