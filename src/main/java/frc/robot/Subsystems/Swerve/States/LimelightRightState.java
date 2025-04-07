package frc.robot.Subsystems.Swerve.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Limelight.LimelightHelpers;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class LimelightRightState extends Command{
    private SwerveSubsystem subsystem;

    boolean forward;
    boolean sideways;
    double aprilTag;


    public LimelightRightState(SwerveSubsystem skibidi)
    {
        super();  
        subsystem = skibidi;
        addRequirements(skibidi);
    }

    @Override
    public void initialize(){

        aprilTag = LimelightHelpers.getFiducialID("limelight-bambino");

        
    }

    @Override
    public void execute(){
        double tx = LimelightHelpers.getTX("limelight-bambino");
        double currentLime = LimelightHelpers.getFiducialID("limelight-bambino");
        
        if(aprilTag == currentLime)
        {
            subsystem.swerveDrivePID(-4.8, tx, 0.6, 0.02, 0.1, false);
        }
           


        
    }

    @Override
    public void end(boolean interrupted)
    {
    }
    
}
