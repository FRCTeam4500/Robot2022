package frc.robot.subsystem.intake;

import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.component.OutputComponent;
import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareIntakeFactory {
    public static Intake makeIntake(){
        OutputComponent motor = new SparkMaxComponent(8, CANSparkMaxLowLevel.MotorType.kBrushless);
        return new IntakeImpl(motor);
    }
}
