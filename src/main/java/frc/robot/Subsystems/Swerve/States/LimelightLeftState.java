package frc.robot.Subsystems.Swerve.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class LimelightLeftState extends Command{
    private SwerveSubsystem subsystem;

    boolean forward;
    boolean sideways;
    double aprilTag;
    boolean done;

    public LimelightLeftState(SwerveSubsystem skibidi)
    {
        super();  
        subsystem = skibidi;
        addRequirements(skibidi);
    }

    @Override
    public void initialize(){

        aprilTag = LimelightHelpers.getFiducialID("limelight-bambino");
        done = false;
        
    }

    @Override
    public void execute(){
        double tx = LimelightHelpers.getTX("limelight-bambino");
        double currentLime = LimelightHelpers.getFiducialID("limelight-bambino");
        
        if(aprilTag == currentLime)
        {
            if(subsystem.swerveDrivePID(24.5, tx, 0.2, 0.08, 0.2, false)){
                done = true;
            }
        }
           


        
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.drive(new Translation2d(0,0), 0, false);
    }

    @Override
    public boolean isFinished(){
        return done;
    }
    
}
