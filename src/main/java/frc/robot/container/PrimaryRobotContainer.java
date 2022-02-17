package frc.robot.container;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.command.AlignWithTargetCommand;
import frc.robot.dashboard.DashboardBooleanDisplay;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.dashboard.DashboardNumberDisplay;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.HardwareArmFactory;
import frc.robot.subsystem.arm.command.ArmDownCommand;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.intake.HardwareIntakeFactory;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.HardwareLoaderFactory;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.HardwareShooterFactory;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.command.ShooterContinuousRunCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.swerve.command.SwerveDefaultCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand;
import frc.robot.subsystem.swerve.command.TriModeSwerveCommand.ControlMode;
import frc.robot.subsystem.swerve.pathfollowingswerve.HardwareSwerveFactory;
import frc.robot.subsystem.swerve.pathfollowingswerve.PathFollowingSwerve;
import frc.robot.subsystem.turret.HardwareTurretFactory;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.turret.command.TurretDefaultCommand;
import frc.robot.subsystem.vision.HardwareVisionFactory;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.util.VisionDistanceCalculator;
import frc.robot.utility.ControllerInfo;


public class PrimaryRobotContainer implements RobotContainer{

    //Initialize subsystems
    private Arm arm = HardwareArmFactory.makeArm();
    private Intake intake = HardwareIntakeFactory.makeIntake();
    private Loader loader = HardwareLoaderFactory.makeLoader();
    private Shooter shooter = HardwareShooterFactory.makeShooter();
    private PathFollowingSwerve swerve = HardwareSwerveFactory.makeSwerve();
    private Turret turret = HardwareTurretFactory.makeTurret();
    private Vision vision = HardwareVisionFactory.makeVision();

    //Initialize Joysticks and Buttons
    private Joystick driveStick = new Joystick(0);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton switchDriveModeRobotCentric = new JoystickButton(driveStick, 1);
    private JoystickButton switchDriveModePolar = new JoystickButton(driveStick, 2);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 5);

    private Joystick controlStick = new Joystick(1);

    private JoystickButton intakeButton = new JoystickButton(controlStick, 1);
    private JoystickButton shootButton = new JoystickButton(controlStick, 3);
    
    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);



    public PrimaryRobotContainer(){
        configureControls();
        configureSwerve();
        configureIntakeAndArm();
        configureShooting();
    }

    void configureControls() {
        info.xSensitivity = 4;
        info.ySensitivity = 4;
        info.zSensitivity = 3.5;
        info.xDeadzone = 0.1;
        info.yDeadzone = 0.1;
        info.zDeadzone = 0.2;
        Shuffleboard.getTab("Driver Controls").add("Driver Controls", info);
        Shuffleboard.getTab("Driver Controls").add("Messages", messages);
    }

    void configureSwerve(){
        TriModeSwerveCommand swerveCommand = new TriModeSwerveCommand(swerve, driveStick, info, vision, turret, messages);
        swerveCommand.controlMode = ControlMode.FieldCentric;
        switchDriveModeRobotCentric.whenPressed(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;});
        switchDriveModeRobotCentric.whenReleased(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;});
        switchDriveModePolar.whenPressed(() -> {swerveCommand.controlMode = ControlMode.Polar;});
        switchDriveModePolar.whenPressed(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;});
        resetGyro.whenPressed(new InstantCommand(() -> {swerve.resetRobotAngle();}));
        swerve.setDefaultCommand(swerveCommand);
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Controls", swerveCommand);

    }

    void configureIntakeAndArm(){
        intakeButton.whenPressed(new ArmSetAngleCommand(arm, ArmConstants.ARM_DOWN_ANGLE)
                .alongWith(new IntakeRunCommand(intake, IntakeConstants.intakeRunSpeed)));
        intakeButton.whenReleased(
            new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).alongWith(
            new IntakeRunCommand(intake, 0))
                        
        );
        Shuffleboard.getTab("Intake").add("Intake", intake);
        Shuffleboard.getTab("Intake").add("Arm", arm);
    }

    void configureShooting() {
        turret.setDefaultCommand(new TurretDefaultCommand(turret, vision));
        shooter.setDefaultCommand(new ShooterContinuousRunCommand(shooter, () -> 0));
        ShooterControl control = new ShooterControl(10000, 50);
        //Command shootCommand = new ManualShootingCommand(shooter, vision, loader, control);
        Command shootCommand = new AutomatedShootingCommand(shooter, vision, loader);
        shootButton.whenPressed(shootCommand);
        shootButton.whenReleased(() -> {shootCommand.cancel(); ; shooter.setSpeed(0); loader.setOutput(0);});
        ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
        tab.add("Shooter", shooter);
        tab.add("Shooter Controls", control);
        tab.add("Turret", turret);
        tab.add("Loader", loader);
        tab.add("Distance", new DashboardNumberDisplay("Distance", () -> VisionDistanceCalculator.calculateDistance(vision)));
    }

    @Override
    public void teleopInit() {
        new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).schedule(); //deploy the arm
    }
}
