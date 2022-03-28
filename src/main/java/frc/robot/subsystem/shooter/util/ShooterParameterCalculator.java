package frc.robot.subsystem.shooter.util;

import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.PolarVelocityCalculator;

/**
     * Class which calculates
     */

    public class ShooterParameterCalculator {


        /**
         * The function for getting shooter speed is
         * f(d) = 442d^2 - 895d + 21219 where d is the horizontal distance from the rim of the target to the robot
         */



        /**
         *
         * @param distance The robot's horizontal distance from the target
         * @return Target speed for the shooter
         */
        public static double getSpeed(double distance){
            double speed = -50*Math.pow(distance, 2) + (2880 * distance) + 17906;
            if (distance < 1.4) //lower port
                speed = 15000;
            return speed;
        }

    /**
     * Calculates distance compensating for movement
     * @param distance
     * @param calculator
     * @return
     */
        public static double getAdjustedDistance(double distance, PolarVelocityCalculator calculator){
            double rVelocity = calculator.getTangentialSpeed();
            double xTranslation = getFlightTime(distance) * rVelocity;
            double adjustedDistance = distance - xTranslation;
            return adjustedDistance;
        }

        public static double getFlightTime(double distance){
            double flightTime = 0.2 * distance + 1;
            return flightTime;
        }


        public static double getTurretOffset(double distance, PolarVelocityCalculator calculator){
            double tVelocity = calculator.getTangentialSpeed();
            double flightTime = getFlightTime(distance);
            double yTranslation = flightTime * tVelocity; //left/right translation
            double turretOffset = Math.atan(yTranslation/distance);

            return turretOffset;
        }

        public static double getTurretOffsetForSideShot(double distance){
            double offset = 0.18; //offset from center of hub in meters
            double hubRadius = 0.6; //radius of hub
            return Math.atan(offset/(distance + hubRadius));
        }
    }

