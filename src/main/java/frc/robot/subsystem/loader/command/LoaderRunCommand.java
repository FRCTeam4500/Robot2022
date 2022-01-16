package frc.robot.subsystem.loader.command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.LoaderConfig;

/**
 * Runs a loader until the command ends
 * We don't need to provide commands for setting the speed because the loader will never be left on, unlike the arm
 */

public class LoaderRunCommand extends CommandBase {
    private Loader loader;

    public LoaderRunCommand(Loader loader){
        this.loader = loader;
        addRequirements(loader);
    }

    public void initialize(){
        loader.setSpeed(LoaderConfig.runSpeed);
    }
}
