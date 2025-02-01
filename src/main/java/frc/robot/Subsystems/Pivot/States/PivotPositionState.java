package frc.robot.Subsystems.Pivot.States;

import java.nio.file.ProviderMismatchException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pivot.ArmSubsystem;

public class PivotPositionState extends Command{
    private ArmSubsystem subsystem;
    private String position;
    private double goal;
    private double limit;
    private double threshold;
    private double armkP;

    public PivotPositionState(ArmSubsystem gyatt, String rizz)
    {
        super();
        this.subsystem = gyatt;
        this.position = rizz;
    }

    @Override
    public void initialize(){
        if(position == "L4"){
            goal = 1;
        }else if(position == "LMid"){
            goal = 2;
        }else if(position == "Home"){
            goal = 3;
        }else if(position == "Algae"){
            goal = 4;
        }else if(position == "Intake"){
            goal = 5;
        }else{
            System.err.println("You Gyatt to get more Rizz");
        }
        threshold = 0.1;
        armkP   = 7/23;
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return subsystem.goToAngle(goal, limit, armkP, threshold);
    }
    
}
