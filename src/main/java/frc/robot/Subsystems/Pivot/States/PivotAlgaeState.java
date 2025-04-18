package frc.robot.Subsystems.Pivot.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotAlgaeState extends Command{
    ArmSubsystem pivot;
    boolean done;
    public PivotAlgaeState(ArmSubsystem skibidi)
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
        if(pivot.pivotPID(17.5, 0.4, 0.2, 0.4))
        {
            done = true;
        }
        pivot.setShootMotor(-1);
        System.out.println("?");
    }

    @Override
    public void end(boolean intterupted){
        pivot.stopPivotMotor();
        pivot.stopShootMotor();

    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
