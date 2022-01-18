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
    private static final HashMap<Double, Pair<Double, Double>> parameters;

    static {
        parameters = new HashMap<Double, Pair<Double, Double>>();
        addParam(0, 0, 0);
    }
    private static void addParam(double distance, double speed, double angle){
        parameters.put(distance, new Pair<Double, Double>(speed, angle));
    }

    /**
     *
     * @param distance The robot's horizontal distance from the target
     * @return a Pair containing the reequired speed and angle for the shooter
     */
    public static Pair<Double, Double> getSpeedAndAngle(double distance){
        Pair<Double, Double> speedAndAngle = new Pair<Double, Double>(0d, 0d);
        Double difference = Double.MAX_VALUE;
        for (Double key : parameters.keySet()) { //finds the closest key (distance) to the given distance
            double newDifference = Math.abs(distance - key);
            if (newDifference < difference) {
                difference = newDifference;
                speedAndAngle = parameters.get(key);
            }
        }
        return speedAndAngle;
    }
}
