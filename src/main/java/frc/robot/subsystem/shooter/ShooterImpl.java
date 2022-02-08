package frc.robot.subsystem.shooter;

import frc.robot.component.AngularVelocityComponent;

public class ShooterImpl implements Shooter{
    private AngularVelocityComponent shooterMotor;
    public ShooterImpl(AngularVelocityComponent motor) {
        shooterMotor = motor;
    }

    public void setSpeed(double speed) {
        shooterMotor.setAngularVelocity(speed / 2); //divided by 2 for gear ratio
    }

    public double getSpeed() {
        return shooterMotor.getAngularVelocity();
    }
}
