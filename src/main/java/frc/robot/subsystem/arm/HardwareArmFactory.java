package frc.robot.subsystem.arm;

import frc.robot.component.hardware.TalonSRXComponent;
import frc.robot.component.AngleComponent;
import frc.robot.component.DoubleMotorComponent;

public class HardwareArmFactory {

    public static Arm makeArm() {
        TalonSRXComponent m1 = new TalonSRXComponent(0);
        TalonSRXComponent m2 = new TalonSRXComponent(0);
        m2.setInverted(true);
        AngleComponent motors = new DoubleMotorComponent(m1, m2);
        Arm arm = new ArmImpl(motors);
        return arm;
    }

}
