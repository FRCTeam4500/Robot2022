package frc.robot.container;


import com.fasterxml.jackson.databind.jsonFormatVisitors.JsonObjectFormatVisitor;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.routines.ConsoleAuto;
import frc.robot.autonomous.routines.FirstBallAuto;
import frc.robot.autonomous.routines.TriangleAuto;
import frc.robot.dashboard.DashboardMessageDisplay;
import frc.robot.dashboard.DashboardNumberDisplay;
import frc.robot.subsystem.arm.Arm;
import frc.robot.subsystem.arm.ArmConstants;
import frc.robot.subsystem.arm.HardwareArmFactory;
import frc.robot.subsystem.arm.command.ArmSetAngleCommand;
import frc.robot.subsystem.camera.CameraImpl;
import frc.robot.subsystem.camera.HardwareCameraFactory;
import frc.robot.subsystem.climber.ClimberConstants;
import frc.robot.subsystem.climber.command.ClimberChainsRunCommand;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import frc.robot.subsystem.intake.HardwareIntakeFactory;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.IntakeConstants;
import frc.robot.subsystem.intake.command.IntakeRunCommand;
import frc.robot.subsystem.loader.HardwareLoaderFactory;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.command.LoaderSetOutputCommand;
import frc.robot.subsystem.shooter.HardwareShooterFactory;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.command.AutomatedShootingCommand;
import frc.robot.subsystem.shooter.command.DumpBallCommand;
import frc.robot.subsystem.shooter.command.ManualShootingCommand;
import frc.robot.subsystem.shooter.command.ShooterSpinUpCommand;
import frc.robot.subsystem.shooter.util.ShooterControl;
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
import frc.robot.utility.PolarVelocityCalculator;


import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.HardwareClimberFactory;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import frc.robot.subsystem.camera.HardwareCameraFactory;
import frc.robot.subsystem.camera.Camera;

public class PrimaryRobotContainer implements RobotContainer{

    //Initialize subsystems
    private Arm arm = HardwareArmFactory.makeArm();
    private Intake intake = HardwareIntakeFactory.makeIntake();
    private Loader loader = HardwareLoaderFactory.makeLoader();
    private Shooter shooter = HardwareShooterFactory.makeShooter();
    private PathFollowingSwerve swerve = HardwareSwerveFactory.makeSwerve();
    private Turret turret = HardwareTurretFactory.makeTurret();
    private Vision vision = HardwareVisionFactory.makeVision();
    private CameraImpl camOne = HardwareCameraFactory.makeCameraInstance();
    private Climber climber = HardwareClimberFactory.makeClimber();
    //private CameraInstance camera = HardwareCameraFactory.makeCameraInstance();

    //Initialize Joysticks and Buttons
    private Joystick driveStick = new Joystick(0);
    private ControllerInfo info = new ControllerInfo();

    private JoystickButton lockSwerveRotationButton = new JoystickButton(driveStick, 1);
    private JoystickButton switchDriveModeRobotCentric = new JoystickButton(driveStick, 2);
    private JoystickButton switchDriveModePolar = new JoystickButton(driveStick, 4);
    private JoystickButton resetGyro = new JoystickButton(driveStick, 5);
    private JoystickButton limitSwerveSpeed = new JoystickButton(driveStick, 4);

    private Joystick controlStick = new Joystick(1);

    private JoystickButton intakeButton = new JoystickButton(controlStick, 1);
    private JoystickButton shootButton = new JoystickButton(controlStick, 3);
    //private JoystickButton reverseLoadButton = new JoystickButton(controlStick, 4);
    private JoystickButton dumpButton = new JoystickButton(controlStick, 5);


    private JoystickButton climberTiltUp = new JoystickButton(controlStick, 12);
    private JoystickButton climberTiltDown = new JoystickButton(controlStick, 11);
    private JoystickButton climberRunChainsForward = new JoystickButton(controlStick, 6);
    private JoystickButton climberRunChainsBackward = new JoystickButton(controlStick, 4);
    
    private DashboardMessageDisplay messages = new DashboardMessageDisplay(15, 50);

    private SendableChooser<Command> autonChooser = new SendableChooser<Command>();

    private PolarVelocityCalculator calculator;
    private TriModeSwerveCommand swerveCommand;

    public PrimaryRobotContainer(){
        configureControls();
        configureClimber();
        configureSwerve();
        configureIntakeAndCameraAndArm();
        configureShooting();
        configureAutonomous();
        configureClimber();
    }

    void configureControls(){
        info.xSensitivity = 4;
        info.ySensitivity = 4;
        info.zSensitivity = 3.5;
        info.xDeadzone = 0.2;
        info.yDeadzone = 0.2;
        info.zDeadzone = 0.2;
        Shuffleboard.getTab("Driver Controls").add("Driver Controls", info);
        Shuffleboard.getTab("Driver Controls").add("Messages", messages);
        //Shuffleboard.getTab("Driver Controls").add("Intake Camera", camera);
    }

    
    

    void configureClimber() {

        Shuffleboard.getTab("Climber").add("climber", climber);
        // When tilt up
        climberTiltUp.whenPressed(new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_UP_ANGLE));

        // When tilt down
        climberTiltDown.whenPressed(new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_DOWN_ANGLE));

        // When chains run forward
        climberRunChainsForward.whenPressed(new ClimberChainsRunCommand(climber, ClimberConstants.CHAINS_RUN_SPEED));
        climberRunChainsForward.whenReleased(new ClimberChainsRunCommand(climber, 0));

        // When chains run backwards
        climberRunChainsBackward.whenPressed(new ClimberChainsRunCommand(climber, -ClimberConstants.CHAINS_RUN_SPEED));
        climberRunChainsBackward.whenReleased(new ClimberChainsRunCommand(climber, 0));
    }

    void configureSwerve() {
        swerveCommand = new TriModeSwerveCommand(swerve, driveStick, info, vision, turret, messages);
        swerveCommand.controlMode = ControlMode.FieldCentric;

        switchDriveModeRobotCentric.whenPressed(() -> {swerveCommand.controlMode = ControlMode.RobotCentric;});
        switchDriveModeRobotCentric.whenReleased(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;});
        switchDriveModePolar.whenPressed(() -> {swerveCommand.controlMode = ControlMode.Polar;});
        switchDriveModePolar.whenPressed(() -> {swerveCommand.controlMode = ControlMode.FieldCentric;});
        lockSwerveRotationButton.whenPressed(() -> {swerveCommand.lockRotation = true;});
        lockSwerveRotationButton.whenReleased(() -> {swerveCommand.lockRotation = false;});

        limitSwerveSpeed.whenPressed(() -> {swerveCommand.limitSpeed = true;});
        limitSwerveSpeed.whenReleased(() -> {swerveCommand.limitSpeed = false;});

        resetGyro.whenPressed(new InstantCommand(() -> {swerve.resetRobotAngle();}));
        swerve.setDefaultCommand(swerveCommand);
        Shuffleboard.getTab("Swerve").add("Swerve", swerve);
        Shuffleboard.getTab("Swerve").add("Swerve Command", swerveCommand);

        calculator = new PolarVelocityCalculator(swerve, vision, turret);
        Shuffleboard.getTab("Swerve").add("Polar Calulator", calculator);

    }

    void configureIntakeAndCameraAndArm() {
        intakeButton.whenPressed(new ArmSetAngleCommand(arm, ArmConstants.ARM_DOWN_ANGLE)
                .alongWith(new IntakeRunCommand(intake, IntakeConstants.intakeRunSpeed)));
        intakeButton.whenReleased(
            new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).alongWith(
            new IntakeRunCommand(intake, 0))
                        
        );

        // configure camera
        camOne.start();
        
        
        Shuffleboard.getTab("Intake").add("Intake", intake);
        Shuffleboard.getTab("Intake").add("Arm", arm);
        Shuffleboard.getTab("Intake").add("Intake Camera", camOne);
    }

    void configureShooting() {
        turret.setDefaultCommand(new TurretDefaultCommand(turret, vision, calculator));
        //shooter.setDefaultCommand(new ShooterContinuousRunCommand(shooter, () -> 0));
        //loader.setDefaultCommand(new LoaderRunCommand(loader, 0));

        //manual shooting
        ShooterControl control = new ShooterControl(10000, 50);
        Command shootCommand = new ManualShootingCommand(shooter, vision, loader, control);
        //shootButton.whenPressed(shootCommand);
        //shootButton.whenReleased(() -> {if (shooter.getCurrentCommand() != null) shooter.getCurrentCommand().cancel(); shooter.setSpeed(0); loader.setOutput(0);});
        //TODO: swap all command cancels with null checked ones
        //Automated shooting
        shootButton.whenPressed(new AutomatedShootingCommand(shooter, vision, loader, calculator).alongWith(new InstantCommand(() -> {swerveCommand.limitSpeed = true;})));
        shootButton.whenReleased(() -> {if (shooter.getCurrentCommand() != null) shooter.getCurrentCommand().cancel(); shooter.setSpeed(0); loader.setOutput(0); swerveCommand.limitSpeed = false;});

        //Run shooter and loader in reverse
        Command reverseLoadCommand = new ParallelCommandGroup(new ShooterSpinUpCommand(shooter, new ShooterControl(10000,50)),
                new LoaderSetOutputCommand(loader, -1));
        //reverseLoadButton.whenPressed(reverseLoadCommand);
        //reverseLoadButton.whenReleased(() -> {loader.getCurrentCommand().cancel(); shooter.setSpeed(0); loader.setOutput(0);});

        Command dumpCommand = new DumpBallCommand(turret, shooter, vision, loader);
        dumpButton.whenPressed(dumpCommand);
        dumpButton.whenReleased(() -> {if (shooter.getCurrentCommand() != null) shooter.getCurrentCommand().cancel(); shooter.setSpeed(0); loader.setOutput(0);});

        //Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Shooting");
        //tab.add("Shooter control", control);
        tab.add("Shooter", shooter);
        tab.add("Turret", turret);
        tab.add("Loader", loader);
        tab.add("Distance", new DashboardNumberDisplay("Distance", () -> VisionDistanceCalculator.calculateDistance(vision)));
    }

   


    void configureAutonomous(){
        autonChooser.setDefaultOption("First Ball", new FirstBallAuto(swerve, arm, shooter, intake, vision, loader));
        autonChooser.addOption("Triangle Auto", new TriangleAuto(swerve, arm, intake, shooter, vision, loader, turret, calculator));
        autonChooser.addOption("Console Auto", new ConsoleAuto(swerve, arm, shooter, intake, vision,loader, turret, calculator));
        Shuffleboard.getTab("Driver Controls").add("Autonomous Route", autonChooser);
    }

    public Command getAutonomousCommand(){
        return autonChooser.getSelected();
    }
    @Override
    public void teleopInit() {
        new ArmSetAngleCommand(arm, ArmConstants.ARM_UP_ANGLE).schedule(); //deploy the arm
    }
}
