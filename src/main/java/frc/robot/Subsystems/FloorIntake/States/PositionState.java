package frc.robot.Subsystems.FloorIntake.States;

import edu.wpi.first.wpilibj2.command.Command;

public class PositionState extends Command
{
    private frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem  subsystem;
    private String position;
    private double goal;
    private double threshold;
    private double limit;
    private double pivotkP;
    private boolean done;

    public PositionState(frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem floorIntake, String WhatIfNinjaGot){
        super();

        this.subsystem = floorIntake;
        this.position  = WhatIfNinjaGot;    
    }

    @Override
    public void initialize(){
        if(position == "Home"){
            goal = 1;
        }else if(position == "Floor"){
            goal = 2;
        }else if(position == "Algae"){
            goal = 3;
        }else if(position == "Trough"){
            goal = 4;
        }else{
            System.err.println("Ninja Did NOT get a low taper fade");
        }
        threshold = 0.1;
        pivotkP   = 7/23;
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.stopFloorPivot();
    }

    @Override
    public boolean isFinished(){
        return subsystem.goToAngle(goal, limit, pivotkP, threshold);
    }
}
