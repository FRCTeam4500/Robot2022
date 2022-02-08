package frc.robot.subsystem.shooter;

import edu.wpi.first.math.util.Units;

/**
 * Constants class for the shooter and its commands
 */

public class ShooterConstants {
    public static final double maximumAllowableOffset = Units.degreesToRadians(5); // maximum allowable vision offset for the shooter to shoot
    public static final double speedThreshold = 400d; //maximum allowable difference between target and actual speeds for the shooter to be considered "at speed"
}
