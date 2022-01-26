package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.components.DoubleAngularVelocityComponent;
import frc.robot.components.hardware.SparkMaxComponent;

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
    

        Shuffleboard.getTab("shooter").add("p", m1.getPIDController().getP());

        /**
         * One motor NEEDS to be inverted! The shooter motors turn in opposite directions
         * If they go the same directions, the shooter will explode.
         */
        m2.setInverted(true);
        DoubleAngularVelocityComponent motors = new DoubleAngularVelocityComponent(m1, m2);
        return new Shooter(motors);
    }
}