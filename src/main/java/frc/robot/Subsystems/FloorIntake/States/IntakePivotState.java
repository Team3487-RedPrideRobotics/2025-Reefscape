
package frc.robot.Subsystems.FloorIntake.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;

public class IntakePivotState extends Command
{
    private DoubleSupplier intakeSpeed;
    private FloorIntakeSubsystem subsystem;

    public IntakePivotState(FloorIntakeSubsystem skibidi, DoubleSupplier balkanRage)
    {
        
        this.intakeSpeed = balkanRage;
        subsystem = skibidi;

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        subsystem.runFloorPivot(intakeSpeed.getAsDouble()*Constants.FloorConstants.FlOOR_PIVOT_SPEED);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopFloorPivot();
    }
    
}