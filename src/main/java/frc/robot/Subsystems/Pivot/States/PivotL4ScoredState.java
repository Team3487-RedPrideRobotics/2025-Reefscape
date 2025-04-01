package frc.robot.Subsystems.Pivot.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotL4ScoredState extends Command{
    ArmSubsystem pivot;
    boolean done;
    public PivotL4ScoredState(ArmSubsystem skibidi)
    {
        super();
        pivot = skibidi;
        
        addRequirements(pivot);
    }

    @Override
    public void initialize(){
        done = false;
    }

    @Override
    public void execute(){
        if(pivot.pivotPID(16, 0.4, 0.2, 0.4))
        {
            done = true;
        }
    }

    @Override
    public void end(boolean intterupted){
        pivot.stopPivotMotor();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
