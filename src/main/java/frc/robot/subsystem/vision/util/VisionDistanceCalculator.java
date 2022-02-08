package frc.robot.subsystem.vision.util;

import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.VisionConstants;

public class VisionDistanceCalculator {
    static double targetHeight = 2.64;


    /**
     * Calculates distance from target using trigonometry
     * @param vision The vision
     * @return the horizontal distance from the target, in meters
     */
    public static double calculateDistance(Vision vision){
        double offset = vision.getVerticalOffsetFromCrosshair();
        double heightDiff = targetHeight - VisionConstants.visionHeight; //opposite side of triangle
        double distance = heightDiff / Math.tan(VisionConstants.visionAngle + offset);
        return distance;
    }
}

