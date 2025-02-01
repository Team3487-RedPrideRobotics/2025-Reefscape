package frc.robot.Subsystems.Elevator.States;

import java.nio.file.ProviderMismatchException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

public class ElevatorPositionState extends Command{
    private ElevatorSubsystem subsystem;
    private String position;
    private double goal;
    private double limit;
    private double threshold;
    private double elevatorkP;

    public ElevatorPositionState(ElevatorSubsystem ohio, String sigma)
    {
        super();
        this.subsystem = ohio;
        this.position = sigma;
    }

    @Override
    public void initialize(){
        if(position == "L4"){
            goal = 1;
        }else if(position == "L3"){
            goal = 2;
        }else if(position == "L2"){
            goal = 3;
        }else if(position == "Home"){
            goal = 4;
        }else if(position == "Intake"){
            goal = 5;
        }else if(position == "High Algae"){
            goal = 4;
        }else if(position == "Low Algae"){
            goal = 5;
        }else{
            System.err.println("The Sigmas got stopped in Ohio");
        }
        threshold  = 0.1;
        elevatorkP = 7/23;
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopMotors();
    }

    @Override
    public boolean isFinished(){
        return subsystem.goToAngle(goal, limit, elevatorkP, threshold);
    }
    
}