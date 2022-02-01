
package frc.robot.component.hardware;

import com.revrobotics.CANSparkMax;

import frc.robot.component.AngularVelocityComponent;
import frc.robot.component.OutputComponent;

/**
 * Wrapper for CANSparkMax motor controllwer which implements SmartMotorComponent
 */
public class SparkMaxComponent extends CANSparkMax implements OutputComponent, AngularVelocityComponent {

    public SparkMaxComponent(int deviceID, MotorType type) {
        super(deviceID, type);
    }

    
    public double getAngle() {
        return -getEncoder().getPosition()*Math.PI * 2;
    }

    
    public void setAngle(double angle) {
        getPIDController().setReference(-angle/2/Math.PI, ControlType.kPosition);

    }

    @Override
    public void setOutput(double output) {
        set(-output);
    }

    @Override
    public double getOutput() {
        return -get();
    }

    @Override
    public double getAngularVelocity() {
        return -getEncoder().getVelocity();
    }

    @Override
    public void setAngularVelocity(double velocity) {
        getPIDController().setReference(-velocity, ControlType.kVelocity);
    }

}
