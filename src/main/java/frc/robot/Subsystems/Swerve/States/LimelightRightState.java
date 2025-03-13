package frc.robot.Subsystems.Swerve.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class LimelightRightState extends Command{
    private SwerveSubsystem subsystem;

    boolean forward;
    boolean sideways;
    double aprilTag;


    public LimelightRightState(SwerveSubsystem skibidi)
    {
        
        subsystem = skibidi;
        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
        forward = false;
        sideways = false;
        aprilTag = LimelightHelpers.getFiducialID("limelight-bambino");

        
    }

    @Override
    public void execute(){
        double tx = LimelightHelpers.getTX("limelight-bambino");
        double ty = LimelightHelpers.getTY("limelight-bambino");
        double currentLime = LimelightHelpers.getFiducialID("limelight-bambino");
        
            if(aprilTag == currentLime)
            {
                if(!forward)
                {
                    System.out.println("elephant");
                    if(subsystem.swerveDrivePID(-2.5, ty, 1, 0.075, 0.1, true))
                    {
                        System.out.println("Thats massive");
                        forward = true;
                    }
                } else {
                    System.out.println("... ... ...DO YOU KNOW WHAT ELSE IS MASSIVE.... LOWWWWWWWWWWWWWWWWW");
                    if(subsystem.swerveDrivePID(22, tx, 0.6, 0.02, 0.1, false))
                    {
                        sideways = true;
                    }
                }
            }
        
       
        
        


        
    }

    @Override
    public void end(boolean interrupted)
    {
    }
    
}
