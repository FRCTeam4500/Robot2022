package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystem.lights.LightRoutine;

public class Rainbow implements LightRoutine {
    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        // For every pixel
        int firstPixelHue = 0;

        for (var i = 0; i < buffer.getLength(); i++) {

            // Calculate the hue - hue is easier for rainbows because the color

            // shape is a circle so only one value needs to precess

            final int hue = (firstPixelHue + (i * 180 / buffer.getLength())) % 180;

            // Set the value

            buffer.setHSV(i, hue, 255, 128);

        }

        // Increase by to make the rainbow "move"

        firstPixelHue += 3;

        // Check bounds

        firstPixelHue %= 180;
    }
}
