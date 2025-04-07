package frc.robot.Subsystems.Elevator.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;
import java.util.prefs.BackingStoreException;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class ElevatorAlgaeTurnState extends Command
{
    private ElevatorSubsystem subsystem;
    boolean done;

    public ElevatorAlgaeTurnState(ElevatorSubsystem skibidi)
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
        if( subsystem.elevatorPID(90, 1, 0.2, 0.4))
        {
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopMotors();
    }

    @Override
    public boolean isFinished(){
        System.out.println(done);
        return done;
    }
    
}