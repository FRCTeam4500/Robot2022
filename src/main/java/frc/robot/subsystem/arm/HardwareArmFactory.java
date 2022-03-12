package frc.robot.subsystem.arm;

import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import frc.robot.component.AngleComponent;
import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareArmFactory {
    public static Arm makeArm(){
        TalonSRXComponent motor = new TalonSRXComponent(9);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configForwardSoftLimitThreshold(ArmConstants.ARM_DOWN_SENSOR_LIMIT);
        motor.configReverseSoftLimitThreshold(ArmConstants.ARM_UP_SENSOR_LIMIT);
        motor.configPeakOutputForward(0.6);
        motor.configPeakOutputReverse(-0.6);
        motor.config_kP(0, 4);
        return new ArmImpl(motor);
    }
}