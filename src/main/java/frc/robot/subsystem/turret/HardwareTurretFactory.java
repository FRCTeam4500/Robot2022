package frc.robot.subsystem.turret;

import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareTurretFactory {
    public static Turret makeTurret(){
        TalonSRXComponent motor = new TalonSRXComponent(10);
        motor.configForwardSoftLimitEnable(true);
        motor.configReverseSoftLimitEnable(true);
        motor.configForwardSoftLimitThreshold(TurretConstants.MAX_SENSOR); //TODO: set these limits
        motor.configReverseSoftLimitThreshold(-TurretConstants.MAX_SENSOR);
        return new TurretImpl(motor);
    }
}
