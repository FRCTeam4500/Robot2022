package frc.robot.subsystem.swerve.command;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;
import frc.robot.utility.ControllerInfo;
import frc.robot.utility.PolarVelocityCalculator;
import edu.wpi.first.util.sendable.Sendable;

/**
 * A swerve command with support for three swerve control modes:
 *
 * Field-Centric:
 * Robot moves relative to the field's axes.
 * When pushing the joystick forward, the robot moves down the field, no matter which way it is facing
 * (Actually, it moves in whatever direction is zeroed to, this just assumes that the gyro is zeroed down the field)
 *
 * Robot-Centric
 * The robot moves relative to itself.
 * When pushing the joystick foward, the robot moves in whatever direction it is facing.
 * For our purposes, the front of the robot is the intake side.
 *
 * AlignToAngle
 * Aligns to a target angle
 * TODO: make this automatically align the robot rotationally with the goal, to avoid the goal going out of view of the vision
 */
public class TriModeSwerveCommand extends CommandBase implements Sendable {
    private Swerve swerve;
    private Joystick joystick;
    private ControllerInfo info;
    private Vision vision;
    private Turret turret;
    private DashboardMessageDisplay messageDisplay;
    private PolarVelocityCalculator polarCalculator;

    private PIDController angleAdjustmentController;
    public ControlMode controlMode;

    public boolean lockRotation = false;
    public boolean limitSpeed = false;
    public double targetAngle = 0;

    private double limitedSpeed = .75;


    public TriModeSwerveCommand(Swerve swerve, Joystick joystick, ControllerInfo controllerInfo, Vision vision, Turret turret, DashboardMessageDisplay messageDisplay){
        this.swerve = swerve;
        this.joystick = joystick;
        info = controllerInfo;
        this.vision = vision;
        this.turret = turret;
        this.messageDisplay = messageDisplay;
        polarCalculator = new PolarVelocityCalculator(swerve, vision, turret);
        controlMode = ControlMode.FieldCentric; //default control mode is field-centric
        angleAdjustmentController = new PIDController(1,0,0);
        angleAdjustmentController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    @Override
    public void execute(){

        double angle = joystick.getPOV();

        if (angle == -1){
            targetAngle = 0;
            controlMode = ControlMode.FieldCentric;
        }

        else if (angle == 0){
            targetAngle = 0;
            controlMode = ControlMode.AlignToAngle;
        }

        else if (angle > 0 && angle < 180){
            targetAngle = -Math.PI/2;
            controlMode = ControlMode.AlignToAngle;
        }

        else if (angle == 180){
            targetAngle = Math.PI;
            controlMode = ControlMode.AlignToAngle;
        }

        else if (angle > 180){
            targetAngle = Math.PI/2;
            controlMode = ControlMode.AlignToAngle;
        }

        double xSpeed = -withDeadzone(joystick.getX(), info.xDeadzone) * info.xSensitivity;
        double ySpeed = -withDeadzone(joystick.getY(), info.yDeadzone) * info.ySensitivity;
        double zSpeed = -withDeadzone(joystick.getZ(), info.zDeadzone) * info.zSensitivity;
        if (limitSpeed){
            xSpeed = ceiling(xSpeed, limitedSpeed);
            ySpeed = ceiling(ySpeed, limitedSpeed);
            zSpeed = ceiling(zSpeed, limitedSpeed);
        }
        if (lockRotation)
            zSpeed = 0;
        switch (controlMode){
            case FieldCentric:
                moveFieldCentric(xSpeed, ySpeed, zSpeed);
                break;
            case RobotCentric:
                moveRobotCentric(xSpeed,ySpeed,zSpeed);
                break;
            case AlignToAngle:
                moveAlign(xSpeed,ySpeed);
                break;
        }
    }

    private void moveFieldCentric(double x, double y, double w){
        swerve.moveFieldCentric(y,x,w);
    }
    private void moveRobotCentric(double x, double y, double w){
        swerve.moveRobotCentric(y,x,w);
    }
    private void moveAlign(double r, double t) {
        double wSpeed = 4 * angleAdjustmentController.calculate(swerve.getRobotAngle(), targetAngle);
        moveFieldCentric(r, t, wSpeed);
    }
    //deadzones the input
    private double withDeadzone(double value, double deadzone){
        if(Math.abs(value) < deadzone)
            return 0;
        else
            return value;
    }

    public enum ControlMode{
        FieldCentric,
        RobotCentric,
        AlignToAngle
    }

    private double ceiling(double value, double maximum){
        if (Math.abs(value) > maximum){
            return maximum * Math.signum(value);
        }
        return value;
    }

    public void initSendable(SendableBuilder builder){
        builder.addStringProperty("Drive Mode", () -> {
            switch (controlMode) {
                case FieldCentric:
                    return "Field Centric";
            
                case RobotCentric:
                    return "Robot Centric";

                case AlignToAngle:
                    return "Align To Angle";
            }
            return "";
        }, null);
        builder.addDoubleProperty("controller x", joystick::getX, null);
        builder.addDoubleProperty("controller y", joystick::getY, null);
        builder.addDoubleProperty("controller z", joystick::getZ, null);
        builder.addBooleanProperty("Limit Speed", () -> {return limitSpeed;}, null);
    }
}
