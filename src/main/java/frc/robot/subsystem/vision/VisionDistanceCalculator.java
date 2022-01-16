package frc.robot.subsystem.vision;

import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.VisionConstants;

public class VisionDistanceCalculator {
    static double targetHeight = 2.64;


    /**
     *
     * @param vision The vision
     * @param height The height of the vision, in meters
     * @param angle The angle above horizon of the vision, in radians
     * @return the horizontal distance from the target, in meters
     */
    public static double calculateDistance(Vision vision, double height, double angle){
        double offset = vision.getVerticalOffsetFromCrosshair();
        double heightDiff = targetHeight - height; //opposite side of triangle
        double distance = heightDiff / Math.tan(angle);
        return distance;
    }
    public static double calculateDistance(Vision vision){
        return calculateDistance(vision, VisionConstants.visionHeight, VisionConstants.visionAngle);
    }
}
