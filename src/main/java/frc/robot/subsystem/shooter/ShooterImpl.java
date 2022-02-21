package frc.robot.subsystem.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.component.AngularVelocityComponent;

public class ShooterImpl implements Shooter{
    private AngularVelocityComponent shooterMotor;
    private double targetSpeed;
    private double threshold;
    public ShooterImpl(AngularVelocityComponent motor) {
        this(motor, 1000);
    }

    public ShooterImpl(AngularVelocityComponent motor, double threshold){
        this.shooterMotor = motor;
        this.threshold = threshold;
    }


    /**
     * Sets the rpm of the shooter
     * @param speed speed, in rpm
     */
    public void setSpeed(double speed) {
        targetSpeed = speed;
        double speedRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(speed);
        shooterMotor.setAngularVelocity(speedRadPerSec / 2); //divided by 2 for gear ratio
    }

    public double getSpeed() {
        return Math.PI * Units.radiansPerSecondToRotationsPerMinute(shooterMotor.getAngularVelocity() * 2);
    }

    public boolean atSpeed(){
        return ((Math.abs(getSpeed() - targetSpeed) <= threshold) && targetSpeed != 0) || getSpeed() > 28500;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("Shooter Threshold", () -> {return threshold;}, (val) -> {threshold = val;});
        builder.addBooleanProperty("Running", () -> !(targetSpeed == 0), null);
        builder.addDoubleProperty("Target Speed", () -> targetSpeed, null);
        builder.addDoubleProperty("Current Speed", this::getSpeed, null);
        builder.addBooleanProperty("at speed", this::atSpeed, null);
    }
}
