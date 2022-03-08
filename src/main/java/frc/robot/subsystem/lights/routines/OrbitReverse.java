package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;

public class OrbitReverse implements LightRoutine {
    private int period = 60;
    private int size;
    private Color color;
    private Color background;

    public OrbitReverse(int size, Color color, Color background){
        this.size = size;
        this.color = color;
        this.background = background;
    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        period = buffer.getLength();
        int currentCycle = period - (cycle % period);
        for (int i = 0; i < buffer.getLength(); i++){
            int actualminus; //i minus size, with rollover if it goes below 0
            if (i - size < 0)
                actualminus = period + i - size;
            else
                actualminus = i - size;
            if (actualminus < currentCycle){
                buffer.setLED(i, color);
            }
            else{
                buffer.setLED(i, background);
            }
        }
    }

    public int getPeriod(){
        return period;
    }
}
