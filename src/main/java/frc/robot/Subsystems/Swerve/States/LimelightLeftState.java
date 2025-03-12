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

    public LimelightLeftState(SwerveSubsystem skibidi)
    {
        
        subsystem = skibidi;
        addRequirements(skibidi);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double tx = LimelightHelpers.getTX("limelight-bambino");
        double ty = LimelightHelpers.getTY("limelight-bambino");
        System.out.println(tx);


        if(tx < 32.5)
        {
            subsystem.drive(new Translation2d(0, -0.2), 0, false);
        }
        
        
        


        
    }

    @Override
    public void end(boolean interrupted)
    {
    }
    
}
