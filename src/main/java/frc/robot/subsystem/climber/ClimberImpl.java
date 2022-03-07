package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.SmartMotorComponent;
import frc.robot.subsystem.arm.ArmConstants;

public class ClimberImpl implements Climber {
    private SmartMotorComponent tiltMotor;
    private SmartMotorComponent chainMotor;
    private double targetTiltAngle = 0; //TODO: Find what angle this is
    private double targetChainOutput = 0; //TODO: LOL

    public ClimberImpl(SmartMotorComponent tiltMotor, SmartMotorComponent chainMotor) {
        this.tiltMotor = tiltMotor;
        this.chainMotor = chainMotor;
    }

    @Override
    public void setPosition(int position) {
        targetTiltAngle = position;
        tiltMotor.setAngle(position);
    }

    @Override
    public void setOutput(double output) {
        targetChainOutput = output;
        chainMotor.setOutput(output);
    }

    public void setTiltOutput(double output){
        tiltMotor.setOutput(output);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Active", () -> !(targetTiltAngle >= 0), null); //TODO: idk how to make it "active" if its either below or above I think this only selects one or the other
        builder.addDoubleProperty("Target tilt position", () -> targetTiltAngle, null);
        builder.addDoubleProperty("Target chain output", () -> targetChainOutput, null);
        builder.addDoubleProperty("current tilt angle", () -> tiltMotor.getAngle(), null);
        builder.addDoubleProperty("current chain output", chainMotor::getOutput, null);
    }
}
