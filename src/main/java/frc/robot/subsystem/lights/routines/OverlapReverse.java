package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;

/**
 * two overlapping colors, going around
 * period is double each length, it must be even
 */
public class OverlapReverse implements LightRoutine {
    private int period;
    private Color color1;
    private Color color2;

    /**
     *
     * @param period must be even
     * @param color1 color 1
     * @param color2 color 2
     */
    public OverlapReverse(int period, Color color1, Color color2){
        this.period = period;
        this.color1 = color1;
        this.color2 = color2;
        if (this.period % 2 != 0) {
            this.period += 1;
        }
    }


    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        int currentCycle = period - (cycle % period); //reverse
        for (int i = 0; i < buffer.getLength(); i++){
            if ((i / (period/2)) % 2 == 0) {
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
