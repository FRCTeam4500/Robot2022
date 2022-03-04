package frc.robot.subsystem.loader.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.LoaderConstants;

/**
 * Runs a loader until the command ends
 * We don't need to provide commands for setting the speed because the loader will never be left on, unlike the arm
 */

public class LoaderSetOutputCommand extends InstantCommand {
    private Loader loader;
    private double speed;

    public LoaderSetOutputCommand(Loader loader){
       this(loader, LoaderConstants.runSpeed);
    }

    public LoaderSetOutputCommand(Loader loader, double output){
        this.loader = loader;
        this.speed = output;
        addRequirements(loader);
    }


    public void initialize(){
        loader.setOutput(speed);
    }
}
