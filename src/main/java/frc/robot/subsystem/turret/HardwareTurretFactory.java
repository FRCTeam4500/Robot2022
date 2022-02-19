package frc.robot.subsystem.turret;

import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareTurretFactory {
    public static Turret makeTurret(){
        TalonSRXComponent motor = new TalonSRXComponent(10);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configForwardSoftLimitThreshold(0); //TODO: set these limits
        motor.configReverseSoftLimitThreshold(0);
        return new TurretImpl(motor);
    }
}
