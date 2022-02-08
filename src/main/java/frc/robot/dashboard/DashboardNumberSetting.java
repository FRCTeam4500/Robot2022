package frc.robot.dashboard;

import edu.wpi.first.networktables.NTSendableBuilder;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DashboardNumberSetting {
    DoubleSupplier number;
    DoubleConsumer setting;
    String name;

    public DashboardNumberSetting (String name, DoubleSupplier number, DoubleConsumer setting){
        this.name = name;
        this.number = number;
        this.setting = setting;
    }

    public void initSendable(NTSendableBuilder builder){
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty(name, number, setting);
    }
}
