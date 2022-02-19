package frc.robot.subsystem.turret.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;

/**
 * Default command for turret
 * Automatically aims it at target
 */

public class TurretDefaultCommand extends CommandBase {
    private Turret turret;
    private Vision vision;
    private PIDController controller;

    public TurretDefaultCommand(Turret turret, Vision vision){
        this.turret = turret;
        this.vision = vision;
        addRequirements(turret);
        controller = new PIDController(1,1,1); //TODO: tune
    }

    public void execute(){
        turret.setOutput(controller.calculate(vision.getHorizontalOffsetFromCrosshair(), 0));
    }
}
