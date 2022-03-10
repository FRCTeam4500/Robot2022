package frc.robot.subsystem.turret.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private ProfiledPIDController controller;

    public TurretDefaultCommand(Turret turret, Vision vision, PolarVelocityCalculator calculator){
        this.turret = turret;
        this.vision = vision;
        this.calculator = calculator;
        addRequirements(turret);
        controller = new ProfiledPIDController(-3.5, 0, 0, new TrapezoidProfile.Constraints(5,1)); 
       // controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(5,1)); 
    }

    public void execute(){
        if (vision.hasValidTargets()){
            double shooterOffset = ShooterParameterCalculator.getTurretOffset(VisionDistanceCalculator.calculateDistance(vision), calculator);
            turret.setOffset(shooterOffset);
            turret.setOutput(controller.calculate(
                    vision.getHorizontalOffsetFromCrosshair(), 0)); //set 0 to turret offset for shoot while move
        }
        else{
            turret.setOffset(0);
            turret.setAngle(0);
        } 
    }
}
