package frc.robot.subsystem.arm;

import frc.robot.component.AngleComponent;

public class HardwareArmFactory {
    public Arm makeArm(){
        AngleComponent motor = null; //TODO: make motor
        return new ArmImpl(motor);
    }
}
