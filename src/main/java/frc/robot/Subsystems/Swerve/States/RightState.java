package frc.robot.Subsystems.Swerve.States;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class RightState extends Command{
    SwerveSubsystem subsystem;
    public RightState(SwerveSubsystem skibidi)
    {
        super();
        subsystem = skibidi;
        addRequirements(subsystem);
    }
    

    @Override
    public void execute(){
        subsystem.drive(new Translation2d(0, 0.5), 0, false);
    }
}
