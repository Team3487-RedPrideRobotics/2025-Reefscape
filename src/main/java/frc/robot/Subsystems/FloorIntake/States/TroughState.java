package frc.robot.Subsystems.FloorIntake.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;
import java.util.prefs.BackingStoreException;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class TroughState extends Command
{
    private FloorIntakeSubsystem subsystem;
    private boolean done;

    public TroughState(FloorIntakeSubsystem skibidi)
    {   
        super();
        subsystem = skibidi;

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        done = false;
    }

    @Override
    public void execute(){
        if(subsystem.FloorPID(5, 0.5, 0.2, 1)){
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopFloorPivot();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
        
}