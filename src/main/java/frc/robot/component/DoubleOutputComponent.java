package frc.robot.component;

public class DoubleOutputComponent implements OutputComponent {
    private OutputComponent c1;
    private OutputComponent c2;
    
    public DoubleOutputComponent(OutputComponent s1, OutputComponent s2) {
        this.c1 = s1;
        this.c2 = s2;
    }

    public void setOutput(double output) {
        c1.setOutput(output);
        c2.setOutput(output);
    }

    public double getOutput(){
        return c1.getOutput();
    }

 

   
    
}
