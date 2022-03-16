package frc.robot.subsystem.climber;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.component.DoubleMotorComponent;
import frc.robot.component.SmartMotorComponent;
import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareClimberFactory {
    public static Climber makeClimber() {

        //SmartMotorComponent motor = new SmartMotorComponent(10); //TODO: Actually create this!
        SparkMaxComponent leftTiltMotor = new SparkMaxComponent(15, CANSparkMaxLowLevel.MotorType.kBrushless);

        SparkMaxComponent rightTiltMotor = new SparkMaxComponent(16, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightTiltMotor.setInverted(false);
        leftTiltMotor.follow(rightTiltMotor, true);


        return new ClimberImpl(rightTiltMotor);
    }
}
