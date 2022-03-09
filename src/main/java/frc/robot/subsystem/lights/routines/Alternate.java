package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;


public class Alternate implements LightRoutine {
    private int period;
    private Color color1;
    private Color color2;


    /**
     *
     * @param period must be even
     * @param color1
     * @param color2
     */
    public Alternate(int period, Color color1, Color color2){
        this.period = period;
        this.color1 = color1;
        this.color2 = color2;
        if (this.period % 2 != 0) {
            this.period += 1;
        }
    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        int currentCycle = cycle % period;
        for (int i = 0; i < buffer.getLength(); i++){
            if ((i / period/2) % 2 == 0) {
                buffer.setLED(i, color1);
            }
            else{
                buffer.setLED(i, color2);
            }
        }
    }

    public int getPeriod(){
        return period;
    }
}
