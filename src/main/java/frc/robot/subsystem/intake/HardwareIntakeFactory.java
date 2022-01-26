package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareIntakeFactory {
    public static Intake makeIntake() {
        SparkMaxComponent m = new SparkMaxComponent(0, MotorType.kBrushless); // TODO: change 0 later
        return new IntakeImpl(m);
    }
}
