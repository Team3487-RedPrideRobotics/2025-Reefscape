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

public class ElevatorBaseState extends Command
{
    private ElevatorSubsystem subsystem;

    public ElevatorBaseState(ElevatorSubsystem skibidi)
    {   
        super();
        subsystem = skibidi;

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        subsystem.elevatorPID(1, 0.8, 0.2, 1);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopMotors();
    }
    
}