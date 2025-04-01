package frc.robot.Subsystems.Pivot.States;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotManualState extends Command
{
    private DoubleSupplier pivotSpeed;
    private ArmSubsystem subsystem;

    public PivotManualState(ArmSubsystem skibidi, DoubleSupplier balkanRage)
    {
        
        this.pivotSpeed = balkanRage;
        subsystem = skibidi;

        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        subsystem.setPivotMotor(pivotSpeed.getAsDouble()*Constants.ArmConstants.PIVOT_SPEED);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopPivotMotor();
    }
    
}