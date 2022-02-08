package frc.robot.subsystem.swerve.pathfollowingswerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.component.hardware.AHRSAngleGetterComponent;
import frc.robot.component.hardware.TalonFXComponent;
import frc.robot.component.hardware.TalonSRXComponent;

public class HardwareSwerveFactory {

    private static final double DRIVE_RATIO = 0; //drive rotations per motor rotation
    private static final double ANGLE_RATIO = 0; //angle rotations per motor rotation
    private static final double MAX_SPEED = 0; //max surface speed, meters per second

    private static final int DBRPORT = 0; //drive back right port
    private static final int ABRPORT = 0; //angle back right port
    private static final int DBLPORT = 0; //drive back right port
    private static final int ABLPORT = 0; //angle back right port
    private static final int DFRPORT = 0; //drive front right port
    private static final int AFRPORT = 0; //angle front right port
    private static final int DFLPORT = 0; //drive front left port
    private static final int AFLPORT = 0; //angle front left port

    private static final double WHEEL_DIAMETER = 0; //Wheel diameter, in meters
    private static final double DRIVE_X_TRANSLATION = 0; //x-axis translation of wheels
    private static final double DRIVE_Y_TRANSLATION = 0; //x-axis translation of wheels

    public static PathFollowingSwerve makeSwerve(){
        OdometricWheelModule fl = makeWheelModule(AFLPORT, DFLPORT, new Translation2d(DRIVE_Y_TRANSLATION / 2, DRIVE_X_TRANSLATION/2), true, true,true);
        OdometricWheelModule fr = makeWheelModule(AFRPORT, DFRPORT, new Translation2d(DRIVE_Y_TRANSLATION / 2, -DRIVE_X_TRANSLATION / 2), true, true,false);
        OdometricWheelModule bl = makeWheelModule(ABLPORT, DBLPORT, new Translation2d(-DRIVE_Y_TRANSLATION / 2, DRIVE_X_TRANSLATION / 2), false, false,true);
        OdometricWheelModule br = makeWheelModule(ABRPORT, DBRPORT, new Translation2d(-DRIVE_Y_TRANSLATION/ 2, -DRIVE_X_TRANSLATION / 2), true, true,false);

        return new OdometricSwerve(
                new AHRSAngleGetterComponent(I2C.Port.kMXP),
                fl,
                fr,
                bl,
                br
        );
    }
    public static OdometricWheelModule makeWheelModule(int angleId, int driveId,Translation2d translationFromSwerveCenter, boolean invertSensorPhase, boolean invertAngle, boolean invertSpeed){
        TalonSRXComponent srx = new TalonSRXComponent(angleId);
        srx.setSensorPhase(invertSensorPhase);
        srx.setInverted(invertAngle);
        srx.config_kP(0, 2);
        srx.config_kF(0,0);
        srx.configMotionCruiseVelocity(5500);
        srx.configMotionAcceleration(5500);
        srx.configAllowableClosedloopError(0, 0);

        TalonFXComponent falcon = new TalonFXComponent(driveId);
        falcon.config_kP(0, 0.03);
        falcon.config_kI(0, 0);
        falcon.config_kD(0,0);
        falcon.config_kF(0, 0.047);
        falcon.config_IntegralZone(0, 0);
        falcon.setInverted(invertSpeed);
        return new OdometricWheelModule(
                srx,
                falcon,
                translationFromSwerveCenter,
                MAX_SPEED,
                WHEEL_DIAMETER,
                ANGLE_RATIO,
                DRIVE_RATIO);
    }
}
