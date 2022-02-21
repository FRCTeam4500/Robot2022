package frc.robot.container;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.routines.FirstBallAuto;
import frc.robot.autonomous.routines.TriangleAuto;
import frc.robot.autonomous.routines.WAuto;
import frc.robot.command.AlignWithTargetCommand;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.dashboard.DashboardNumberDisplay;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.HardwareArmFactory;
import frc.robot.subsystem.arm.command.ArmDownCommand;
import frc.robot.subsystem.intake.HardwareIntakeFactory;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.HardwareLoaderFactory;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.HardwareShooterFactory;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
import frc.robot.subsystem.swerve.command.SwerveDefaultCommand;
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

    private JoystickButton switchDriveMode = new JoystickButton(driveStick, 1);

    private Joystick controlStick = new Joystick(1);

    private JoystickButton intakeButton = new JoystickButton(controlStick, 1);
    private JoystickButton shootButton = new JoystickButton(controlStick, 2);

    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();



    public PrimaryRobotContainer(){
        configureControls();
        configureSwerve();
        configureIntakeAndArm();
        configureShooting();
        configureAutonomous();
    }

    void configureControls() {
        info.xSensitivity = 1;
        info.ySensitivity = 1;
        info.zSensitivity = 1;
        info.xDeadzone = 0.1;
        info.yDeadzone = 0.1;
        info.zDeadzone = 0.1;
        Shuffleboard.getTab("Driver Controls").add("Controller Info", info);
    }

    void configureSwerve(){
        SwerveDefaultCommand swerveCommand = new SwerveDefaultCommand(swerve, driveStick, info);
        switchDriveMode.whenPressed(() -> {swerveCommand.isRobotCentric = true;});
        switchDriveMode.whenReleased(() -> {swerveCommand.isRobotCentric = false;});
        swerve.setDefaultCommand(swerveCommand);
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
    }

    void configureIntakeAndArm(){
        Command intakeCommand = new IntakeRunCommand(intake);
        Command armCommand = new ArmDownCommand(arm);
        intakeButton.whenHeld(new ParallelCommandGroup(intakeCommand,armCommand));
        Shuffleboard.getTab("Intake").add("Intake", intake);
        Shuffleboard.getTab("Intake").add("Arm", arm);
    }

    void configureShooting() {
        turret.setDefaultCommand(new TurretDefaultCommand(turret, vision));
        ShooterControl control = new ShooterControl(10000, 50);
        Command shootCommand = new ManualShootingCommand(shooter, vision, loader, control);
        shootButton.whenHeld(shootCommand);
        ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
        tab.add("Shooter", shooter);
        tab.add("Shooter Controls", control);
        tab.add("Turret", turret);
        tab.add("Loader", loader);
        tab.add("Distance", new DashboardNumberDisplay("Distance", () -> VisionDistanceCalculator.calculateDistance(vision)));
    }

    void configureAutonomous(){
        autonChooser.addOption("First Ball", new FirstBallAuto(swerve, arm, shooter, intake, vision, loader));
        autonChooser.addOption("Triangle Auto", new TriangleAuto(swerve, arm, intake, shooter, vision, loader));
        autonChooser.addOption("W Auto", new WAuto(swerve, arm, intake, shooter, vision, loader));
        Shuffleboard.getTab("Driver Controls").add("Autonomous Route", autonChooser);
    }

    public Command getAutonomousCommand(){
        return autonChooser.getSelected();
    }
}
