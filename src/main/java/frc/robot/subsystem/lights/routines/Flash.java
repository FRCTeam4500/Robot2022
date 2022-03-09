package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystem.lights.LightRoutine;
import frc.robot.subsystem.lights.LightUtils;

public class Flash implements LightRoutine {
    private int period;
    private Color color1;
    private Color color2;

    public Flash(int period, Color color1, Color color2){
        this.period = period;
        this.color1 = color1;
        this.color2 = color2;
    }

    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        if ((cycle / period) % period == 0){
            LightUtils.setEntireBuffer(buffer, color1);
        }
        else{
            LightUtils.setEntireBuffer(buffer, color2);
        }
    }

    public int getPeriod(){
        return period;
    }
}
