package frc.robot.subsystem.shooter.util;
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
    }

