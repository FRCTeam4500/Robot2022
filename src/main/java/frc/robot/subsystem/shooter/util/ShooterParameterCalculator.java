package frc.robot.subsystem.shooter.util;
    /**
     * Class which calculates
     */

    public class ShooterParameterCalculator {


        /**
         * The function for getting shooter speed is
         * f(d) = 6710d + 7430, where d is the distance in meters from the rim of the goal
         * f(d) = 3399d + 15671 if the shooter does actually max out at 28500
         */



        /**
         *
         * @param distance The robot's horizontal distance from the target
         * @return Target speed for the shooter
         */
        public static double getSpeed(double distance){
            double speed = 3399*distance + 15671;
            return speed;
        }
    }

