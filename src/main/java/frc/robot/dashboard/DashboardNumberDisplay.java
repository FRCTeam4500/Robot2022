package frc.robot.dashboard;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.DoubleSupplier;

public class DashboardNumberDisplay implements NTSendable {
    DoubleSupplier number;
    String name;
    public DashboardNumberDisplay (String name, DoubleSupplier number){
        this.name = name;
        this.number = number;
    }

    public void initSendable(NTSendableBuilder builder){
        builder.addDoubleProperty(name, number, null);
    }
}
