package frc.robot.subsystem.turret.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.shooter.util.ShooterParameterCalculator;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.util.VisionDistanceCalculator;
import frc.robot.utility.PolarVelocityCalculator;

/**
 * Default command for turret
 * Automatically aims it at target
 */

public class TurretDefaultCommand extends CommandBase {
    private Turret turret;
    private Vision vision;
    private PolarVelocityCalculator calculator;
    private PIDController controller;

    public TurretDefaultCommand(Turret turret, Vision vision, PolarVelocityCalculator calculator){
        this.turret = turret;
        this.vision = vision;
        this.calculator = calculator;
        addRequirements(turret);
        controller = new PIDController(-1.8,0,0); 
    }

    public void execute(){
        if (vision.hasValidTargets()){
            turret.setOutput(controller.calculate(
                    vision.getHorizontalOffsetFromCrosshair(), ShooterParameterCalculator.getTurretOffset(VisionDistanceCalculator.calculateDistance(vision), calculator)));
        }
        else{
            turret.setAngle(0);
        } 
    }
}
