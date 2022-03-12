package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.DoubleMotorComponent;
import frc.robot.component.SmartMotorComponent;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.component.DoubleMotorRunOppositeComponent;

public class ClimberImpl implements Climber {
    private SmartMotorComponent tiltMotor; //TODO: Find out how hooks for other bars work to implement
    private double targetTiltAngle = 0; //TODO: Find what angle this is

    public ClimberImpl(SmartMotorComponent tiltMotor) {
        this.tiltMotor = tiltMotor;
    }

    @Override
    public void setPosition(int position) {
        targetTiltAngle = position;
        tiltMotor.setAngle(position);
    }

    
    public void setTiltOutput(double output){
        tiltMotor.setOutput(output);
    }

   

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, null);
    }
}
