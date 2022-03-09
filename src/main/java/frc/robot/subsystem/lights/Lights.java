package frc.robot.subsystem.lights;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.lights.routines.*;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.subsystem.lights.LightUtils.*;


public class Lights extends SubsystemBase {

    private AddressableLED lights;
    private AddressableLEDBuffer buffer;
    private int size;
    private int cycle = 0;
    private int cycleAdjust = 0;
    private int period = 1;

    private LightRoutine currentLightRoutine;


    public Lights(int period, int port, int size){
        initMap();
        currentLightRoutine = lightRoutines.get(Routines.rainbow);
        lights = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(size);
        this.size = size;
    }

    public void periodic(){
        increment();
        currentLightRoutine.updateLEDData(buffer,cycleAdjust);
        lights.setData(buffer);
    }

    void increment(){
        cycle++;
        if (cycle % period == 0){
            cycleAdjust++;
        }
        if (cycle >= 10000)
            cycle = 0;
        if (cycleAdjust >= size * currentLightRoutine.getPeriod()){ //reset cycle synchronized with the size of the light strip and period of the routine to avoid cutting off the routine
            cycleAdjust = 0;
        }
    }

    enum Routines{
        rainbow,
        blueorange,
        blueorangereverse,
        bluesine,
        orangesinereverse,
        blueflash,
        orangeorbit,
        redflash,
        random,

    }

    public void setCurrentRoutine(Routines routine){
        currentLightRoutine = lightRoutines.get(routine);
    }

    private Map<Routines, LightRoutine> lightRoutines = new HashMap<Routines, LightRoutine>();

    private void initMap(){
        lightRoutines.put(Routines.rainbow, new Rainbow());
        lightRoutines.put(Routines.blueorange, new Overlap(6, claytonBlueBright, claytonOrangeBright));
        lightRoutines.put(Routines.blueorangereverse, new OverlapReverse(6, claytonOrangeBright,claytonBlueBright));
        lightRoutines.put(Routines.bluesine, new Sine(10, claytonBlueBright));
        lightRoutines.put(Routines.orangesinereverse, new SineReverse(10, claytonOrangeBright));
        lightRoutines.put(Routines.random, new Random());
        lightRoutines.put(Routines.blueflash, new Flash(10, claytonBlueBright, wpiCol(0,0,0)));
        lightRoutines.put(Routines.orangeorbit, new Orbit(5, claytonOrangeBright, new Color(0,0,0)));
        lightRoutines.put(Routines.redflash,new Flash(10, wpiCol(255,0,0), wpiCol(0,0,0)));
    }


}
