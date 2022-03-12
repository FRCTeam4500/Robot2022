package frc.robot.subsystem.climber;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Climber extends Subsystem, Sendable {
    void setPosition(int position);
    //void setOutput(double output);
    void setTiltOutput(double output);
    //void setChainPosition(int position);
}
