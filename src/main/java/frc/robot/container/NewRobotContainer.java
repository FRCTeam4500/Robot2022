package frc.robot.container;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystem.TankDrive.TankDrive;
import frc.robot.subsystem.TankDrive.TankDriveFactory;
import frc.robot.subsystem.TankDrive.TankDriveCommand;
import frc.robot.subsystem.shooter.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystem.intake.HardwareIntakeFactory;
import frc.robot.subsystem.arm.HardwareArmFactory;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.arm.command.ArmDownCommand;


public class NewRobotContainer implements RobotContainer {
    private Joystick driveStick = new Joystick(0);
    private TankDrive tankDrive = TankDriveFactory.makeTankDrive();
    private double xSensitivity = 1, ySensitivity = 1, zSensitivity = 4, xDeadzone = 0.2, yDeadzone = 0.2, zDeadzone = 0.3;
    private Shooter shooter;
    private ShooterControl shooterControl;
    private Intake intake;
    private Arm arm;
    private JoystickButton shooterButton = new JoystickButton(driveStick, 2);
    private JoystickButton intakeButton = new JoystickButton(driveStick, 1);

    public NewRobotContainer() { 
        
        configureTankDrive();
        configureShooter();

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
        Shuffleboard.getTab("Shooter").add("Shooter", new ShooterShuffleboardControls(shooterControl));
    }

    public void configureIntakeArm() {
        intake = HardwareIntakeFactory.makeIntake();
        arm = HardwareArmFactory.makeArm();
        intakeButton.whileHeld(new IntakeRunCommand(intake).alongWith(new ArmDownCommand(arm)));
    
    }

}
