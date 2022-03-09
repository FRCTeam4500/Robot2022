package frc.robot.subsystem.climber.sequence;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import frc.robot.subsystem.climber.command.ClimberSetChainPositionCommand;

public class ClimbToHighBar extends SequentialCommandGroup {
    Climber climber;

    public ClimbToHighBar(Climber climber){
        this.climber = climber;
        addCommands(
                new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_CLIMB_ANGLE),
                new WaitCommand(1),
                new ClimberSetChainPositionCommand(climber, 0) //TODO: get this number
        );
    }


}
