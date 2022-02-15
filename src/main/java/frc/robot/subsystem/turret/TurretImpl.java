package frc.robot.subsystem.turret;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.AngleComponent;
import frc.robot.component.SmartMotorComponent;

public class TurretImpl implements Turret{
    private SmartMotorComponent motor;
    private double targetAngle;
    public TurretImpl(SmartMotorComponent motor){
        this.motor = motor;
    }

    /**
     * sets turret angle, in radians. If the given angle is larger than the maximum anggle, sets to maximum
     * @param angle target angle in radians
     */
    public void setAngle(double angle){
        if (Math.abs(angle) > TurretConstants.MAX_ANGLE){
            targetAngle = Math.signum(angle) * TurretConstants.MAX_ANGLE;
        }
        else{
            targetAngle = angle;
        }
        motor.setAngle(targetAngle / TurretConstants.TURRET_RATIO);
    }

    public double getAngle(){
        return motor.getAngle() * TurretConstants.TURRET_RATIO;
    }

    /**
     * moves the turret
     * @param output Percent output of the motor
     */
    public void setOutput(double output){
        motor.setOutput(output);
    }

    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Target Angle", () -> targetAngle, null);
        builder.addDoubleProperty("Current Angle", () -> motor.getAngle(), null);
    }
}
