package frc.robot.Subsystems.Swerve.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight.LimelightHelpers;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class LimelightForwardState extends Command{
    private SwerveSubsystem subsystem;
    
    double aprilTag;
    boolean done;


    public LimelightForwardState(SwerveSubsystem skibidi)
    {
        
        subsystem = skibidi;
        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        aprilTag = LimelightHelpers.getFiducialID("limelight-bambino");
        done = false;
        System.out.println("EWJKEBFIOUQWEFHUIEOJFOIJWEOIFJIOWEJFIOWE");
    }

    @Override
    public void execute(){
        double ty = LimelightHelpers.getTY("limelight-bambino");
        double currentLime = LimelightHelpers.getFiducialID("limelight-bambino");
        
            if(aprilTag == currentLime)
            {
                if(subsystem.swerveDrivePID(22, ty, 1, 0.075, 0.2, true))
                {
                    System.out.println("Thats massive");
                    done = true;
                }
                 
            }

        if(ty == 0){
            done = true;
        }
    }
        
       
        
        

    
    

    @Override
    public void end(boolean interrupted)
    {
        subsystem.drive(new Translation2d(0,0), 0, false);

    }

    @Override 
    public boolean isFinished(){
        System.out.println("ahh:" + done);
        return done;
    }
    
}
