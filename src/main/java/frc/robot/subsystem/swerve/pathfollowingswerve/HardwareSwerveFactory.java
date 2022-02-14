package frc.robot.subsystem.swerve.pathfollowingswerve;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.component.hardware.TalonFXComponent;
import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareSwerveFactory {

    private static final double DRIVE_RATIO = 1/4.3329; //drive rotations per motor rotation
    private static final double ANGLE_RATIO = 1/12.34567901234; //angle rotations per motor rotation
    private static final double MAX_SPEED = 4.8; //max surface speed, meters per second

    private static final int DBRPORT = 2; //drive back right port
    private static final int ABRPORT = 3; //angle back right port
    private static final int DBLPORT = 12; //drive back right port
    private static final int ABLPORT = 11; //angle back right port
    private static final int DFRPORT = 4; //drive front right port
    private static final int AFRPORT = 5; //angle front right port
    private static final int DFLPORT = 7; //drive front left port
    private static final int AFLPORT = 6; //angle front left port

    private static final double WHEEL_DIAMETER = 0.0762; //Wheel diameter, in meters
    private static final double DRIVE_X_TRANSLATION = 0.2921; //x-axis translation of left wheels
    private static final double DRIVE_Y_TRANSLATION = 0.2794; //x-axis translation of right wheels

    public static PathFollowingSwerve makeSwerve(){
        OdometricWheelModule fl = makeWheelModule(AFLPORT, DFLPORT, new Translation2d(DRIVE_Y_TRANSLATION, DRIVE_X_TRANSLATION), true, true,true);
        OdometricWheelModule fr = makeWheelModule(AFRPORT, DFRPORT, new Translation2d(DRIVE_Y_TRANSLATION, -DRIVE_X_TRANSLATION), true, true,false);
        OdometricWheelModule bl = makeWheelModule(ABLPORT, DBLPORT, new Translation2d(-DRIVE_Y_TRANSLATION, DRIVE_X_TRANSLATION), false, true,true);
        OdometricWheelModule br = makeWheelModule(ABRPORT, DBRPORT, new Translation2d(-DRIVE_Y_TRANSLATION, -DRIVE_X_TRANSLATION ), true, true,false);

        return new OdometricSwerve(
                new AHRSAngleGetterComponent(I2C.Port.kMXP),
                fl,
                fr,
                bl,
                br
        );
    }
    public static OdometricWheelModule makeWheelModule(int angleId, int driveId,Translation2d translationFromSwerveCenter, boolean invertSensorPhase, boolean invertAngle, boolean invertSpeed){
        TalonFXComponent angleMotor = new TalonFXComponent(angleId);
        angleMotor.setSensorPhase(invertSensorPhase);
        angleMotor.setInverted(invertAngle);
        angleMotor.config_kP(0, 0.5);
        angleMotor.config_kF(0,0);
        angleMotor.configMotionCruiseVelocity(5500);
        angleMotor.configMotionAcceleration(5500);
        angleMotor.configAllowableClosedloopError(0, 0);
        angleMotor.configClearPositionOnQuadIdx(true, 10);

        TalonFXComponent driveMotor = new TalonFXComponent(driveId);
        driveMotor.config_kP(0, 0.03);
        driveMotor.config_kI(0, 0);
        driveMotor.config_kD(0,0);
        driveMotor.config_kF(0, 0.047);
        driveMotor.config_IntegralZone(0, 0);
        driveMotor.setInverted(invertSpeed);
        return new OdometricWheelModule(
                angleMotor,
                driveMotor,
                translationFromSwerveCenter,
                MAX_SPEED,
                WHEEL_DIAMETER,
                ANGLE_RATIO,
                DRIVE_RATIO);
    }
}
