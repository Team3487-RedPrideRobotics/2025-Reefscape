package frc.robot.Subsystems.Swerve.States;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;


public class DriveToPose extends frc.robot.Utils.State<SwerveSubsystem> {
    
    private SwerveSubsystem subsystem;
    private Command command;
    private Supplier<Pose2d> pose;
    
    public DriveToPose(SwerveSubsystem skibidi, Pose2d targetPose){
        super(skibidi);
        this.subsystem = skibidi;

        pose = () -> targetPose;
    }

    @Override
    public void initialize(){
        command = AutoBuilder.pathfindToPose(pose.get(),
         new PathConstraints(
            Constants.AutonConstants.AUTO_MAX_SPEED, 
            Constants.AutonConstants.AUTO_MAX_ACCELERATION, 
            Constants.AutonConstants.AUTO_MAX_ROTATION_SPEED.getDegrees(),
            Constants.AutonConstants.AUTO_MAX_ROTATION_ACCELERATION.getDegrees()
            )
        );

        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }
    
    @Override
    public void end(boolean interrupted)
    {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished(){
        return command.isFinished();
    }
    
}
