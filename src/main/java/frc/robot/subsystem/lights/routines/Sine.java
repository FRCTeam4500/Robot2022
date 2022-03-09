package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;
import frc.robot.subsystem.lights.LightUtils;


public class Sine implements LightRoutine {
    private int period = 10;
    private int hue;

    public Sine(int period, Color color){
        this.period = period;
        this.hue = LightUtils.getHue((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)) / 2;

    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        int currentCycle = cycle % period;
        for (int i = 0; i < buffer.getLength(); i++){
            buffer.setHSV((i + currentCycle) % buffer.getLength(), hue, 255, (int) (255 * ((Math.sin((Math.PI / (period / 2)) * currentCycle)) + 1) / 2));
        }
    }


    public int getPeriod(){
        return period;
    }


}
