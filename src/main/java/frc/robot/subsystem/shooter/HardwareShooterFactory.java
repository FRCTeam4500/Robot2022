package frc.robot.subsystem.shooter;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.component.AngularVelocityComponent;
import frc.robot.component.DoubleMotorComponent;
import frc.robot.component.hardware.SparkMaxComponent;

public class HardwareShooterFactory {
    public static Shooter makeShooter(){
        SparkMaxComponent m1 = new SparkMaxComponent(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxComponent m2 = new SparkMaxComponent(2, CANSparkMaxLowLevel.MotorType.kBrushless);

        m1.getPIDController().setP(0.0001);
        m2.getPIDController().setP(0.0001);
        
        //m1.getPIDController().setP(0.001);
        //m2.getPIDController().setP(0.001);
        //m2.getPIDController().setFF(0.00000481);
        //m2.getPIDController().setFF(0.00000481);
    

        /**
         * One motor NEEDS to be inverted! The shooter motors turn in opposite directions
         * If they go the same directions, the shooter will explode.
         */
        m2.setInverted(true);
        AngularVelocityComponent motors = new DoubleMotorComponent(m1,m2);
        return new ShooterImpl(motors);
    }
}
