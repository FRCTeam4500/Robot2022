package frc.robot.subsystem.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.component.AngularVelocityComponent;

public class Shooter extends SubsystemBase {
    private AngularVelocityComponent shooterMotor;
    public Shooter(AngularVelocityComponent motor) {
        shooterMotor = motor;
    }

    public void setSpeed(double speed) {
        shooterMotor.setAngularVelocity(speed / 2); //divided by 2 for gear ratio
    }

    public double getSpeed() {
        return shooterMotor.getAngularVelocity();
    }

}
