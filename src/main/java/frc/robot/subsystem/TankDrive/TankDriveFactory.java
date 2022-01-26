package frc.robot.subsystems.TankDrive;

import frc.robot.components.hardware.VictorSPComponent;
import frc.robot.components.DoubleOutputSetterComponent;
public class TankDriveFactory {
    public static TankDrive makeTankDrive() {
        VictorSPComponent left1 = new VictorSPComponent(1);
        VictorSPComponent left2 = new VictorSPComponent(2);
        VictorSPComponent right1 = new VictorSPComponent(3);
        VictorSPComponent right2 = new VictorSPComponent(4);
        DoubleOutputSetterComponent left = new DoubleOutputSetterComponent(left1, left2);
        DoubleOutputSetterComponent right = new DoubleOutputSetterComponent(right1, right2);
        TankDrive tankDrive = new TankDrive(left, right);
        return tankDrive;
    }
}
