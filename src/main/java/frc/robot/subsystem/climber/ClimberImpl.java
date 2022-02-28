package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.SmartMotorComponent;
import frc.robot.subsystem.arm.ArmConstants;

public class ClimberImpl implements Climber {
    private SmartMotorComponent smartMotor;
    private double targetAngle = 0; //TODO: Find what angle this is
    private double targetOutput = 0; //TODO: LOL

    public ClimberImpl(SmartMotorComponent smartMotor) { this.smartMotor = smartMotor;}

    @Override
    public void setAngle(double angle) {
        targetAngle = angle;
        smartMotor.setAngle(angle);
    }

    @Override
    public void setOutput(double output) {
        targetOutput = output;
        smartMotor.setOutput(output);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("Active", () -> !(targetAngle >= ClimberConstants.CLIMBER_UP_ANGLE), null);
        builder.addDoubleProperty("Target angle", () -> targetAngle, null);
        builder.addDoubleProperty("Target output", () -> targetOutput, null);
        builder.addDoubleProperty("currentAngle", () -> smartMotor.getAngle(), null);
        builder.addDoubleProperty("currentOutput", () -> smartMotor.getOutput(), null);
    }

}
