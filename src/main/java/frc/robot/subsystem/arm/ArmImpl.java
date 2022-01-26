package frc.robot.subsystem.arm;

import frc.robot.component.AngleComponent;

public class ArmImpl implements Arm {
    
    private AngleComponent motor;    

    public ArmImpl(AngleComponent motor) {
        this.motor = motor;
    }

    public void setAngle(double angle) {
        motor.setAngle(angle);
    }

}
