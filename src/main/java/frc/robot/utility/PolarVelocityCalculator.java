package frc.robot.utility;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystem.swerve.Swerve;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.vision.Vision;

public class PolarVelocityCalculator {
    private Vision vision;
    private Swerve swerve;
    private Turret turret;
    private double pi = Math.PI;

    public PolarVelocityCalculator(Swerve swerve, Vision vision, Turret turret){
        this.vision = vision;
        this.swerve = swerve;
        this.turret = turret;
    }

    public Pair<Double, Double> getPolarSpeeds(){
        double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
        ChassisSpeeds speeds = swerve.getSpeeds();
        /**
         * translates speeds from robot xy directions to tangential and radial directions
         * it does this by scalar projection of the xy vectors onto the polar vectors
         * A = angle between robot x axis and the target
         * vr = vxcosA  + vycos(90-A)
         * vt = vxcos(90-A) + vycos(A)
         */
        double radialVelocity = (speeds.vxMetersPerSecond * Math.cos(angleFromTarget)) +
                (speeds.vyMetersPerSecond * Math.cos(pi/2 - angleFromTarget));
        double tangentialVelocity = (speeds.vxMetersPerSecond * Math.cos(pi/2 - angleFromTarget)) +
                (speeds.vyMetersPerSecond * Math.cos(angleFromTarget));
        return new Pair<> (radialVelocity, tangentialVelocity);
    }

    public double getRadialSpeed(){
       ChassisSpeeds speeds = swerve.getSpeeds();
       double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
       return (speeds.vxMetersPerSecond * Math.cos(angleFromTarget)) + (speeds.vyMetersPerSecond * Math.cos(pi/2 - angleFromTarget));
    }

    public double getTangentialSpeed(){
        ChassisSpeeds speeds = swerve.getSpeeds();
        double angleFromTarget = pi/2 - (vision.getVisionAngle() + turret.getAngle()); //angle between the robot's x axis and the target, will be pi/2 if the robot is facing the target
        return (speeds.vxMetersPerSecond * Math.cos(pi/2 - angleFromTarget)) + (speeds.vyMetersPerSecond * Math.cos(angleFromTarget));
    }
}
