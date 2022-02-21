package frc.robot.subsystem.swerve.command;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
 * Polar:
 * The robot moves in polar space with the goal as the center.
 * When pushing the joystick forwards, the robot moves towards the goal
 * When pushing the joystick right/left, the robot goes counter/clockwise around the goal.
 * This only works when the robot has a vision target, the robot will not move if it doesn't.
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

    /**
     * Adjusts the angle of the robot with the goal of making the turret angle equal to 0.
     * This would mean that the front of the robot is directly facing the target.
     * This is so that the target does not go out of the field of vision while driving in polar mode.
     */
    private PIDController polarAngleAdjustmentController;
    public ControlMode controlMode;

    public boolean lockRotation = false;


    public TriModeSwerveCommand(Swerve swerve, Joystick joystick, ControllerInfo controllerInfo, Vision vision, Turret turret, DashboardMessageDisplay messageDisplay){
        this.swerve = swerve;
        this.joystick = joystick;
        info = controllerInfo;
        this.vision = vision;
        this.turret = turret;
        this.messageDisplay = messageDisplay;
        polarCalculator = new PolarVelocityCalculator(swerve, vision, turret);
        controlMode = ControlMode.FieldCentric; //default control mode is field-centric
        /**
        * TODO: tune this loop so the robot aligns with the target at the right speed
         * The robot MUST turn slower then and in the opposite direction of the turret,
         * otherwise there will be a positive feedback loop and the bot will go into an out of control spin cycle,
         * then most likely disassemble itself in a way akin to that of a dryer with a brick in it.
         */
        polarAngleAdjustmentController = new PIDController(-0.5,0,0);
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        double xSpeed = -withDeadzone(joystick.getX(), info.xDeadzone) * info.xSensitivity;
        double ySpeed = -withDeadzone(joystick.getY(), info.yDeadzone) * info.ySensitivity;
        double zSpeed = -withDeadzone(joystick.getZ(), info.zDeadzone) * info.zSensitivity;
        if (lockRotation)
            zSpeed = 0;
        switch (controlMode){
            case FieldCentric:
                moveFieldCentric(xSpeed, ySpeed, zSpeed);
                break;
            case RobotCentric:
                moveRobotCentric(xSpeed,ySpeed,zSpeed);
                break;
            case Polar:
                movePolar(xSpeed,ySpeed,zSpeed);
                break;
        }
    }

    private void moveFieldCentric(double x, double y, double w){
        swerve.moveFieldCentric(y,x,w);
    }
    private void moveRobotCentric(double x, double y, double w){
        swerve.moveRobotCentric(y,x,w);
    }
    private void movePolar(double r, double t, double w){
        if(vision.hasValidTargets()) {
            Pair<Double, Double> speeds = polarCalculator.calculateCartesianSpeeds(r, t);
            double xSpeed = speeds.getFirst();
            double ySpeed = speeds.getSecond();
            /**
             * Attempts to move the robot so that the angle between the turret and robot are 0, at which point the robot is directly facing the target.
             * This assumes that the turret is always seeking out the target, that is, that the angle between turret and target is zero.
             */
            double wSpeed = polarAngleAdjustmentController.calculate(turret.getAngle(), 0);
            moveRobotCentric(ySpeed, xSpeed, wSpeed);
        }
        else{
            /**
             * If the vision has no targets, the robot will move robot-centric in x and y,
             * but will continue to try to align itself with the turret,
             * in the hopes that it regains sight of the target.
             */
            messageDisplay.addMessage("Could not move polar-ly, No vision targets found", this);
            double wSpeed = polarAngleAdjustmentController.calculate(turret.getAngle(), 0);
            moveRobotCentric(r,t,wSpeed);
        }
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
        Polar
    }

    public void initSendable(SendableBuilder builder){
        builder.addStringProperty("Drive Mode", () -> {
            switch (controlMode) {
                case FieldCentric:
                    return "Field Centric";
            
                case RobotCentric:
                    return "Robot Centric";

                case Polar:
                    return "Polar";
            }
            return "";
        }, null);
        builder.addDoubleProperty("controller x", joystick::getX, null);
        builder.addDoubleProperty("controller y", joystick::getY, null);
        builder.addDoubleProperty("controller z", joystick::getZ, null);
    }
}
