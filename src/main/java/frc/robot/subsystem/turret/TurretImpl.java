package frc.robot.subsystem.turret;

import frc.robot.component.AngleComponent;

public class TurretImpl implements Turret{
    private AngleComponent motor;
    public TurretImpl(AngleComponent motor){
        this.motor = motor;
    }

    public void setAngle(double angle){
        motor.setAngle(angle / TurretConstants.TURRET_ROT_PER_MOTOR_ROT);
    }

    public double getAngle(){
        return motor.getAngle() * TurretConstants.TURRET_ROT_PER_MOTOR_ROT;
    }
}
