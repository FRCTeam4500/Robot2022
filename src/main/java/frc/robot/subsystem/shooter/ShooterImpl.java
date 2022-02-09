package frc.robot.subsystem.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.AngularVelocityComponent;

public class ShooterImpl implements Shooter{
    private AngularVelocityComponent shooterMotor;
    private double targetSpeed;
    public ShooterImpl(AngularVelocityComponent motor) {
        shooterMotor = motor;
    }


    /**
     * Sets the rpm of the shooter
     * @param speed speed, in rpm
     */
    public void setSpeed(double speed) {
        targetSpeed = speed;
        shooterMotor.setAngularVelocity(speed / 2); //divided by 2 for gear ratio
    }

    public double getSpeed() {
        return shooterMotor.getAngularVelocity() * 2;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Running", () -> !(targetSpeed == 0), null);
        builder.addDoubleProperty("Target Speed", () -> targetSpeed, null);
        builder.addDoubleProperty("Current Speed", this::getSpeed, null);
    }
}
