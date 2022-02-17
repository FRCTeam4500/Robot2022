package frc.robot.subsystem.shooter.util;
    /**
     * Class which calculates
     */

    public class ShooterParameterCalculator {


        /**
         * The function for getting shooter speed is
         * f(d) = 6710d + 7430, where d is the distance in meters from the rim of the goal
         */



        /**
         *
         * @param distance The robot's horizontal distance from the target
         * @return Target speed for the shooter
         */
        public static double getSpeed(double distance){
            double speed = 6710*distance + 7430;
            return speed;
        }
    }

