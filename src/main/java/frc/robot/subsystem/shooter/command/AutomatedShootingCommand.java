package frc.robot.subsystem.shooter.command;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystem.loader.Loader;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.util.ShooterParameterCalculator;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.turret.command.LockTurretConditionalCommand;
import frc.robot.subsystem.vision.Vision;
import frc.robot.subsystem.vision.util.VisionDistanceCalculator;
import frc.robot.subsystem.loader.command.LoaderRunConditionalCommand;
import frc.robot.utility.PolarVelocityCalculator;

public class AutomatedShootingCommand extends SequentialCommandGroup {
    private Shooter shooter;
    private Vision vision;
    private Loader loader;
    private Turret turret;
    private boolean stopUpdate;
    private double prevDist = 0;


    public AutomatedShootingCommand(Shooter shooter, Vision vision, Loader loader, Turret turret, PolarVelocityCalculator calculator){
        this.shooter = shooter;
        this.vision = vision;
        this.loader = loader;
        this.turret = turret;
        
        Supplier<Double> distance = () -> { //stop updating distance when the shooter is at speed
            if (!stopUpdate)
                setPrevDist(VisionDistanceCalculator.calculateDistance(vision)); //update if update is not disabled
            if (shooter.atSpeed())
                setStopUpdate(true); //stop updating when turret is at speed
            return prevDist;
        };
        
        addRequirements(shooter, loader);
        addCommands(
            new InstantCommand(() -> {turret.setEnabled(true); setStopUpdate(false);}),
            new ParallelCommandGroup(
                new ShooterContinuousRunCommand(shooter, () -> {return ShooterParameterCalculator.getSpeed(distance.get());}),
                new LockTurretConditionalCommand(turret, () -> stopUpdate),
                new LoaderRunConditionalCommand(loader, shooter::atSpeed) // () -> {return shooter.atSpeed();}  shoots when shooter is at speed
        ));
    }
    public void end(){
        turret.setEnabled(true);
        setStopUpdate(false);
        loader.setOutput(0);
        shooter.setSpeed(0);
    }

    public void interrupted(){
        end();
    }

    void setStopUpdate(boolean stop){
        stopUpdate = stop;
    }

    void setPrevDist(double prevDist){
        this.prevDist = prevDist;
    }
}
