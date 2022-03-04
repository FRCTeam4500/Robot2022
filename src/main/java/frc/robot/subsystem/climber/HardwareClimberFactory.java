package frc.robot.subsystem.climber;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.component.SmartMotorComponent;
import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareClimberFactory {
    public static Climber makeClimber() {

        //SmartMotorComponent motor = new SmartMotorComponent(10); //TODO: Actually create this!
        SparkMaxComponent angleMotor = new SparkMaxComponent(90, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxComponent outputMotor = new SparkMaxComponent(80, CANSparkMaxLowLevel.MotorType.kBrushless);

        return new ClimberImpl(angleMotor, outputMotor);
    }
}
