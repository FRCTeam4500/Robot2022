package frc.robot.subsystem.arm;

import frc.robot.component.AngleComponent;

public class ArmImpl implements Arm{
    private AngleComponent angleComponent;

    public ArmImpl(AngleComponent motor){
        angleComponent = motor;
    }

    @Override
    public void setAngle(double angle) {
        angleComponent.setAngle(angle);
    }
}
