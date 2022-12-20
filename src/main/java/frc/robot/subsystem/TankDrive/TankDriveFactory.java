package frc.robot.subsystem.TankDrive;

import frc.robot.component.DoubleOutputComponent;
import frc.robot.component.hardware.VictorSPComponent;
import frc.robot.component.hardware.TalonSRXComponent;

public class TankDriveFactory {
    public static TankDrive makeTankDrive() {
        TalonSRXComponent Left = new TalonSRXComponent(1);
        TalonSRXComponent Right = new TalonSRXComponent(2);
        TankDrive tankDrive = new TankDrive(Left, Right);
        return tankDrive;
    }
}
