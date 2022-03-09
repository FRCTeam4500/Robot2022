package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;
import frc.robot.subsystem.lights.LightUtils;


public class Sine implements LightRoutine {
    private int period = 10;
    private int hue;
    private boolean reverse;
    private int timeScale;

    public Sine(int period, Color color, boolean reverse, int timeScale){
        this.period = period;
        this.hue = LightUtils.getHue((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)) / 2;
        this.reverse = reverse;
        this.timeScale = timeScale;

    }

    public Sine(int period, Color color, boolean reverse){
        this(period, color, reverse, 1);
    }

    public Sine(int period, Color color, int timeScale){
        this(period,color,false, timeScale);
    }

    public Sine(int period, Color color){
        this(period, color, false, 1);
    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        cycle /= timeScale;
        int currentCycle = (cycle)% period;
        if (reverse)
            currentCycle = period - (cycle % period); //reverse
        for (int i = 0; i < buffer.getLength(); i++){
            buffer.setHSV((i + currentCycle) % buffer.getLength(), hue, 255, (int) (255 * ((Math.sin((Math.PI / (period / 2)) * currentCycle)) + 1) / 2));
        }
    }


    public int getPeriod(){
        return period;
    }


}
