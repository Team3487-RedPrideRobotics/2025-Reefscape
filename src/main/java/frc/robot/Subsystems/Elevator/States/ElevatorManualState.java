package frc.robot.Subsystems.Elevator.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;
import java.util.prefs.BackingStoreException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class ElevatorManualState extends Command
{
    private ElevatorSubsystem subsystem;
    private DoubleSupplier elevatorSpeed;
    private DigitalInput beamBreak;


    public ElevatorManualState(ElevatorSubsystem skibidi, DoubleSupplier balkanRage)
    {
        
        subsystem = skibidi;
        elevatorSpeed = balkanRage;

        beamBreak = new DigitalInput(0);

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
    
    if(!beamBreak.get() || (beamBreak.get() && elevatorSpeed.getAsDouble()<0)){
    subsystem.runMotors(elevatorSpeed.getAsDouble());
    }

    
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopMotors();
    }

    
    
}