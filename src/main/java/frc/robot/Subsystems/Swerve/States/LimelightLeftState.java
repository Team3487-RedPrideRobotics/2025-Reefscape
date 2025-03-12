package frc.robot.Subsystems.Swerve.States;

import java.util.function.DoubleSupplier;

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
        double tx = LimelightHelpers.getTX("limelight-bambino");
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted)
    {
    }
    
}
