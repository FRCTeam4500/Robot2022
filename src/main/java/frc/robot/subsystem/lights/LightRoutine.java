package frc.robot.subsystem.lights;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LightRoutine {
    void updateLEDData(AddressableLEDBuffer buffer, int cycle);
    int getPeriod();
    
}
