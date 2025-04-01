package frc.robot.Subsystems.Swerve.States;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class BackwardTimedState extends Command{
    SwerveSubsystem subsystem;
    Timer timer;
    double startTime;
    double endTime;
    double waitTime;
    boolean done;

    public BackwardTimedState(SwerveSubsystem skibidi, double wait)
    {
        super();
        subsystem = skibidi;
        waitTime = wait;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize(){
        startTime = timer.get();
        endTime = startTime + waitTime;
        done = false;
    }
 
    @Override
    public void execute(){
        if(timer.get() <= endTime){
            subsystem.drive(new Translation2d(0.5, 0), 0, false);
        } else {
            done = true;
        }
    }

    @Override 
    public boolean isFinished(){
        return done;
    }
}
