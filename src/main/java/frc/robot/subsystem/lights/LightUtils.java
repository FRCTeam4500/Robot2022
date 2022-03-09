package frc.robot.subsystem.lights;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LightUtils {

    public static int getHue(int red, int green, int blue) {

        float min = Math.min(Math.min(red, green), blue);
        float max = Math.max(Math.max(red, green), blue);

        if (min == max) {
            return 0;
        }

        float hue = 0f;
        if (max == red) {
            hue = (green - blue) / (max - min);

        } else if (max == green) {
            hue = 2f + (blue - red) / (max - min);

        } else {
            hue = 4f + (red - green) / (max - min);
        }

        hue = hue * 60;
        if (hue < 0) hue = hue + 360;

        return Math.round(hue);
    }


    public static Color wpiCol(int red, int green, int blue){
        return new Color(red / 255d, green / 255d, blue / 255d);
    }

    public static void setEntireBuffer(AddressableLEDBuffer buffer, Color color){
        for (int i = 0; i < buffer.getLength(); i++){
            buffer.setLED(i, color);
        }
    }

    public static final Color claytonBlue = wpiCol(0,74,128);
    public static final Color claytonOrange = wpiCol(243,108,33);
    public static final Color claytonBlueBright = wpiCol(0,148,255);
    public static final Color claytonOrangeBright = wpiCol(255,118,35);
}
