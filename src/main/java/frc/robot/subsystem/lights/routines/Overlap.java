package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;



public class Overlap implements LightRoutine {
    private int period;
    private Color color1;
    private Color color2;
    private boolean reverse;
    private int timeScale;


    /**
     *
     * @param period must be even
     * @param color1
     * @param color2
     */
    public Overlap(int period, Color color1, Color color2, boolean reverse, int timeScale){
        this.period = period;
        this.color1 = color1;
        this.color2 = color2;
        this.reverse = reverse;
        this.timeScale = timeScale;
        if (this.period % 2 != 0) {
            this.period += 1;
        }
    }

    public Overlap(int period, Color color1, Color color2, int timeScale){
        this(period, color1, color2, false, timeScale);
    }

    public Overlap(int period, Color color1, Color color2, boolean reverse){
        this(period, color1, color2, reverse, 1);
    }

    public Overlap(int period, Color color1, Color color2){
        this(period, color1, color2, false, 1);
    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        cycle /= timeScale;
        int currentCycle = cycle % period;
        if (reverse)
            currentCycle = period - (cycle % period); //reverse
        for (int i = 0; i < buffer.getLength(); i++){
            if ((i / period/2) % 2 == 0) {
                buffer.setLED((i + currentCycle) % buffer.getLength(), color1);
            }
            else{
                buffer.setLED((i + currentCycle) % buffer.getLength(), color2);
            }
        }
    }

    public int getPeriod(){
        return period;
    }
}
