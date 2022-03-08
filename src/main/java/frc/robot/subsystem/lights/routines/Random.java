package frc.robot.subsystem.lights.routines;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystem.lights.LightRoutine;

public class Random implements LightRoutine {
    public void updateLEDData(AddressableLEDBuffer buffer, int cycle){
        for (int i = 0; i < buffer.getLength(); i++){
            buffer.setHSV(i, (int) (Math.random() * 180), 255, 255);
        }
    }

    public int getPeriod(){
        return 1;
    }
}
