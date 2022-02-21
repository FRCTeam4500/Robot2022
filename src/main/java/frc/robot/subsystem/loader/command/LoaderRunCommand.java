package frc.robot.subsystem.loader.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.LoaderConstants;

/**
 * Runs a loader until the command ends
 * We don't need to provide commands for setting the speed because the loader will never be left on, unlike the arm
 */

public class LoaderRunCommand extends CommandBase {
    private Loader loader;
    private double speed;

    public LoaderRunCommand(Loader loader){
       this(loader, LoaderConstants.runSpeed);
    }

    public LoaderRunCommand(Loader loader, double output){
        this.loader = loader;
        this.speed = output;
        addRequirements(loader);
    }

    public void initialize(){
        loader.setOutput(speed);
    }
}
