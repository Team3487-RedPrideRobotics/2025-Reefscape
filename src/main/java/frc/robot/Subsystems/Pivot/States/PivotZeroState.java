package frc.robot.Subsystems.Pivot.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotZeroState extends Command{
    ArmSubsystem pivot;
    public PivotZeroState(ArmSubsystem skibidi)
    {
        super();
        pivot = skibidi;
        addRequirements(pivot);
    }

    @Override
    public void execute(){
        pivot.pivotPID(0, 0.4, 0.2, 0.4);
    }

    @Override
    public void end(boolean intterupted){
        pivot.stopPivotMotor();
    }

    @Override
    public boolean isFinished(){
        return pivot.pivotPID(0, 0.4, 0.2, 0.4);
    }
}
