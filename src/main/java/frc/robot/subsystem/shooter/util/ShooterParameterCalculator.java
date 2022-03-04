package frc.robot.subsystem.shooter.util;

import frc.robot.subsystem.swerve.Swerve;
import frc.robot.utility.PolarVelocityCalculator;

/**
     * Class which calculates
     */

    public class ShooterParameterCalculator {


        /**
         * The function for getting shooter speed is
         * f(d) = 781d^2 - 1151d + 22502 where d is the horizontal distance from the rim of the target to the robot
         */



        /**
         *
         * @param distance The robot's horizontal distance from the target
         * @return Target speed for the shooter
         */
        public static double getSpeed(double distance){
            double speed = 781*Math.pow(distance, 2) - (1151 * distance) + 22502;
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
    }

