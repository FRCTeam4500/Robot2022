package frc.robot.container;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.DashboardNumberDisplay;
import frc.robot.component.hardware.LimelightVisionComponent;
import frc.robot.subsystem.TankDrive.TankDrive;
import frc.robot.subsystem.TankDrive.TankDriveFactory;
import frc.robot.subsystem.TankDrive.TankDriveCommand;
import frc.robot.subsystem.shooter.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystem.shooter.command.ShooterSpinDownCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinUpCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.VisionDistanceCalculator;
import frc.robot.subsystem.vision.VisionImpl;


public class NewRobotContainer implements RobotContainer {
    private Joystick driveStick = new Joystick(0);
    private TankDrive tankDrive = TankDriveFactory.makeTankDrive();
    private double xSensitivity = 1, ySensitivity = 1, zSensitivity = 4, xDeadzone = 0.2, yDeadzone = 0.2, zDeadzone = 0.3;
    private Shooter shooter;
    private ShooterControl shooterControl;
    private JoystickButton shooterButton = new JoystickButton(driveStick, 1);
    private JoystickButton shooterOff = new JoystickButton(driveStick, 2);
    private Vision vision;





    public NewRobotContainer() { 
        
        configureTankDrive();
        configureShooter();
        configureVision();

    }

    public void configureTankDrive() {
        TankDriveCommand tankCommand = new TankDriveCommand(tankDrive, driveStick, xDeadzone, xSensitivity);
        tankDrive.setDefaultCommand(tankCommand);
        
    }

    public void configureShooter() {
        shooterControl = new ShooterControl();
        shooter = HardwareShooterFactory.makeShooter();
        shooterButton.whileHeld(new ShooterSpinUpCommand(shooter, shooterControl));
        shooterButton.whenReleased(new ShooterSpinDownCommand(shooter));
        Shuffleboard.getTab("Shooting").add("Shooter Parameters", new ShooterShuffleboardControls(shooterControl));
    }
    public void configureVision(){
        vision = new VisionImpl(new LimelightVisionComponent());
        Shuffleboard.getTab("Shooting").add(new DashboardNumberDisplay("Distance from rim",
                () -> {return VisionDistanceCalculator.calculateDistance(vision);}));
    }
}
