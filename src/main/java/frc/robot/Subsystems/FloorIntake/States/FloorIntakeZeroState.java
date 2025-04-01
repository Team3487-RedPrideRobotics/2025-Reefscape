
package frc.robot.Subsystems.FloorIntake.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;

public class FloorIntakeZeroState extends Command
{
    private DoubleSupplier intakeSpeed;
    private FloorIntakeSubsystem subsystem;

    public FloorIntakeZeroState(FloorIntakeSubsystem skibidi)
    {
        
        subsystem = skibidi;

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        subsystem.FloorPID(1, 0.5, 0.2, 1);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopFloorPivot();
    }
    
}