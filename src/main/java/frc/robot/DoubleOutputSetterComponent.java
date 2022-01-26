package frc.robot.components;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.components.OutputSetterComponent;

public class DoubleOutputSetterComponent implements OutputSetterComponent {
    private OutputSetterComponent c1;
    private OutputSetterComponent c2;
    
    public DoubleOutputSetterComponent(OutputSetterComponent s1, OutputSetterComponent s2) {
        this.c1 = s1;
        this.c2 = s2;
    }

    public void setOutput(double output) {
        c1.setOutput(output);
        c2.setOutput(output);
    }

 

   
    
}
