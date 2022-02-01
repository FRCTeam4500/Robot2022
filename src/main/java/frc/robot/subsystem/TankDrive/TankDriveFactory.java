package frc.robot.subsystem.TankDrive;

import frc.robot.component.DoubleOutputComponent;
import frc.robot.component.hardware.VictorSPComponent;
public class TankDriveFactory {
    public static TankDrive makeTankDrive() {
        VictorSPComponent left1 = new VictorSPComponent(1);
        VictorSPComponent left2 = new VictorSPComponent(2);
        VictorSPComponent right1 = new VictorSPComponent(3);
        VictorSPComponent right2 = new VictorSPComponent(4);
        DoubleOutputComponent left = new DoubleOutputComponent(left1, left2);
        DoubleOutputComponent right = new DoubleOutputComponent(right1, right2);
        TankDrive tankDrive = new TankDrive(left, right);
        return tankDrive;
    }
}
