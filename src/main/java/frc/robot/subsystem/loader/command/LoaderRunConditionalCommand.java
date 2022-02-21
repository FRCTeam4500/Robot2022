package frc.robot.subsystem.loader.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.dashboard.DashboardBooleanDisplay;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.loader.LoaderConstants;

/**
 * Runs a loader until the command ends
 * We don't need to provide commands for setting the speed because the loader will never be left on, unlike the arm
 */

public class LoaderRunConditionalCommand extends CommandBase {
    private Loader loader;
    private BooleanSupplier condition;

    public LoaderRunConditionalCommand(Loader loader, BooleanSupplier condition){
        this.loader = loader;
        this.condition = condition;
        //Shuffleboard.getTab("Shooting").add("Loader condition", new DashboardBooleanDisplay("loader condition met", condition));
        addRequirements(loader);
    }

    public void initialize(){
        loader.setOutput(0); // Change to 1
    }

    public void execute(){
        if (condition.getAsBoolean()){
            loader.setOutput(LoaderConstants.runSpeed);
        } 
        else{
            loader.setOutput(0); // Change to 0
        }
    }
}
