package frc.robot.subsystem.climber.sequence;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.climber.ClimberConstants;
import frc.robot.subsystem.climber.command.ClimberSetAngleCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ClimbToHighBar extends ParallelCommandGroup {
    Climber climber;

    public ClimbToHighBar(Climber climber){
        this.climber = climber;
        
        /**addCommands(
                new ClimberSetAngleCommand(climber, ClimberConstants.CLIMBER_CLIMB_ANGLE),
                //new WaitCommand(1),
                new SequentialCommandGroup(new WaitCommand(2), new ClimberSetChainPositionCommand(climber, ClimberConstants.CLIMBER_CHAIN_ANGLE)
                //new ClimberSetChainPositionCommand(climber, ClimberConstants.CLIMBER_CHAIN_ANGLE) //TODO: get this number
        ));*/
        
    }


}
