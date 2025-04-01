package frc.robot.Subsystems.Pivot.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotIntakeState extends Command{
    ArmSubsystem pivot;
    public PivotIntakeState(ArmSubsystem skibidi)
    {
        super();
        pivot = skibidi;
        addRequirements(pivot);
    }

    @Override
    public void execute(){
        pivot.pivotPID(9, 0.4, 0.2, 0.4);
        pivot.setShootMotor(1);
    }

    @Override
    public void end(boolean intterupted){
        pivot.stopPivotMotor();
        pivot.stopShootMotor();
    }
}
