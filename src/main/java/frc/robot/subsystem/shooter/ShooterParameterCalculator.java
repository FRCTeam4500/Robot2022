package frc.robot.subsystem.shooter;


import edu.wpi.first.math.Pair;

import java.util.HashMap;

/**
 * Class which calculates
 */

public class ShooterParameterCalculator {


    /**
     * HashMap containing parameters for different distances for the shooter
     * The key is the distance
     * The first item of the pair is the shooter speed for that distance
     * The second item is the shooter angle for that distance.
     */
    private static final HashMap<Double, Double> parameters;

    static {
        parameters = new HashMap<Double, Double>();
        addParam(0, 0);
    }
    private static void addParam(double distance, double speed){
        parameters.put(distance, speed);
    }

    /**
     *
     * @param distance The robot's horizontal distance from the target
     * @return a Pair containing the reequired speed and angle for the shooter
     */
    public static double getSpeed(double distance){
        Double speed = 0d;
        Double difference = Double.MAX_VALUE;
        for (Double key : parameters.keySet()) { //finds the closest key (distance) to the given distance
            double newDifference = Math.abs(distance - key);
            if (newDifference < difference) {
                difference = newDifference;
                speed = parameters.get(key);
            }
        }
        return speed;
    }
}
