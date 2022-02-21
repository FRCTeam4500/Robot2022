package frc.robot.subsystem.arm;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.AngleComponent;

public class ArmImpl implements Arm{
    private AngleComponent angleComponent;
    private double targetAngle = ArmConstants.ARM_UP_ANGLE;

    public ArmImpl(AngleComponent motor){
        this.angleComponent = motor;
    }

    @Override
    public void setAngle(double angle) {
        targetAngle = angle;
        angleComponent.setAngle(angle);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("Active", () -> !(targetAngle >= ArmConstants.ARM_UP_ANGLE), null);
        builder.addDoubleProperty("Target angle", () -> targetAngle, null);
        builder.addDoubleProperty("currentAngle", () -> angleComponent.getAngle(), null);
    }
}
