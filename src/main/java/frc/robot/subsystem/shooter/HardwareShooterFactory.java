package frc.robot.subsystem.shooter;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.component.AngularVelocityComponent;
import frc.robot.component.DoubleMotorComponent;
import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareShooterFactory {
    public static Shooter makeShooter(){
        SparkMaxComponent motor = new SparkMaxComponent(14, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.getPIDController().setP(0.0001);

        return new ShooterImpl(motor);
    }
}
