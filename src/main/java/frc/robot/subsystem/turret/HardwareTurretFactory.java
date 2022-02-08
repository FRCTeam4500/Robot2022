package frc.robot.subsystem.turret;

import frc.robot.component.AngleComponent;

public class HardwareTurretFactory {
    public static Turret makeTurret(){
        AngleComponent motor = null; //TODO: make motor and configure max and min angles
        return new TurretImpl(motor);
    }
}
