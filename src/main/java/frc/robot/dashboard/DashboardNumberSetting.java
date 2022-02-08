package frc.robot.dashboard;

import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DashboardNumberSetting {
    Supplier<Double> number;
    Consumer<Double> setting;
    String name;

    public DashboardNumberSetting (String name, Supplier<Double> number, Consumer<Double> setting){
        this.name = name;
        this.number = number;
        this.setting = setting;
    }

    public void initSendable(NTSendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty(name, (DoubleSupplier) number, (DoubleConsumer) setting);
    }
}
