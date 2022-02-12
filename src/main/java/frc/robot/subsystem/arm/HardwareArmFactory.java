package frc.robot.subsystem.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import frc.robot.component.AngleComponent;
import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareArmFactory {
    public static Arm makeArm(){
        TalonSRXComponent motor = new TalonSRXComponent(9);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configForwardSoftLimitThreshold(0);
        motor.configReverseSoftLimitThreshold(ArmConstants.ARM_DOWN_SENSOR_LIMIT);
        return new ArmImpl(motor);
    }
}
