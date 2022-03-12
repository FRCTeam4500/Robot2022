package frc.robot.subsystem.climber;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.component.DoubleMotorComponent;
import frc.robot.component.SmartMotorComponent;
import frc.robot.component.hardware.SparkMaxComponent;
import frc.robot.component.DoubleMotorRunOppositeComponent;

public class HardwareClimberFactory {
    public static Climber makeClimber() {

        //SmartMotorComponent motor = new SmartMotorComponent(10); //TODO: Actually create this!
        SparkMaxComponent leftTiltMotor = new SparkMaxComponent(15, CANSparkMaxLowLevel.MotorType.kBrushless);

        SparkMaxComponent rightTiltMotor = new SparkMaxComponent(16, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightTiltMotor.setInverted(true);

        DoubleMotorComponent TiltMotors = new DoubleMotorComponent(leftTiltMotor, rightTiltMotor);

        return new ClimberImpl(TiltMotors);
    }
}
