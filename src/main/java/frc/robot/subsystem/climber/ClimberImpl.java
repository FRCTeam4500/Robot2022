package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.SmartMotorComponent;
import frc.robot.subsystem.arm.ArmConstants;

public class ClimberImpl implements Climber {
    private SmartMotorComponent smartMotor;
    private SmartMotorComponent smartMotor2;
    private double targetAngle = 0; //TODO: Find what angle this is
    private double targetOutput = 0; //TODO: LOL

    public ClimberImpl(SmartMotorComponent smartMotor, SmartMotorComponent smartMotor2) {
        this.smartMotor = smartMotor;
        this.smartMotor2 = smartMotor2;
    }

    @Override
    public void setAngle(double angle) {
        targetAngle = angle;
        smartMotor.setAngle(angle);
    }

    @Override
    public void setOutput(double output) {
        targetOutput = output;
        smartMotor2.setOutput(output);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addBooleanProperty("Active", () -> !(targetAngle >= 0), null); //TODO: idk how to make it "active" if its either below or above I think this only selects one or the other
        builder.addDoubleProperty("Target angle", () -> targetAngle, null);
        builder.addDoubleProperty("Target output", () -> targetOutput, null);
        builder.addDoubleProperty("currentAngle", () -> smartMotor.getAngle(), null);
        builder.addDoubleProperty("currentOutput", () -> smartMotor2.getOutput(), null);
    }

}
