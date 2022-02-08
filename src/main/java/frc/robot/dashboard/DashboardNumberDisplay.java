package frc.robot.dashboard;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DashboardNumberDisplay implements NTSendable {
    Supplier<Double> number;
    String name;
    public DashboardNumberDisplay (String name, Supplier<Double> number){
        this.name = name;
        this.number = number;
    }

    public void initSendable(NTSendableBuilder builder){
        builder.addDoubleProperty(name, (DoubleSupplier) number, null);
    }
}
