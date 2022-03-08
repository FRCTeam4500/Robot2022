package frc.robot.subsystem.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.lights.routines.Rainbow;

import java.util.HashMap;
import java.util.Map;


public class Lights extends SubsystemBase {

    private AddressableLED lights;
    private AddressableLEDBuffer buffer;
    private int cycle = 0;
    private int cycleAdjust = 0;
    private int period = 1;

    private Routine currentRoutine = Routine.rainbow;


    public Lights(int period, int port, int size){
        initMap();
        lights = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(size);
    }

    public void periodic(){
        increment();
        routines.get(currentRoutine).updateLEDData(buffer,cycleAdjust);
        lights.setData(buffer);
    }

    void increment(){
        cycle++;
        if (cycle % period == 0){
            cycleAdjust++;
        }
        if (cycle >= 10000)
            cycle = 0;
        if (cycleAdjust >= 10000){
            cycle = 0;
        }
    }

    enum Routine{
        rainbow
    }

    private Map<Routine, LightRoutine> routines = new HashMap<Routine, LightRoutine>();

    private void initMap(){
        routines.put(Routine.rainbow, new Rainbow());
    }


}
