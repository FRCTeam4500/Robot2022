package frc.robot.subsystem.intake;

import frc.robot.component.OutputComponent;

public class HardwareIntakeFactory {
    public static Intake makeIntake(){
        OutputComponent motor = null; //TODO: add motor
        return new IntakeImpl(motor);
    }
}
