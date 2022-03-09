package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;

public class Orbit implements LightRoutine {
    private int period = 60;
    private int size;
    private Color color;
    private Color background;
    private boolean reverse;
    private int timeScale;



    public Orbit(int size, Color color, Color background, boolean reverse, int timeScale){
        this.size = size;
        this.color = color;
        this.background = background;
        this.reverse = reverse;
        this.timeScale = timeScale;
    }

    public Orbit(int size, Color color, Color background, boolean reverse){
        this(size, color, background, reverse, 1);
    }


    public Orbit(int size, Color color, Color background, int timeScale){
        this(size, color, background, false, timeScale);
    }


    public Orbit(int size, Color color, Color background){
        this(size, color, background, false, 1);
    }



    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        cycle /= timeScale;
        period = buffer.getLength();
        int currentCycle = cycle % period;
        if (reverse)
            currentCycle = period - (cycle % period);
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
