package frc.robot.Subsystems.Limelight;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Utils.Loggable;
import frc.robot.Utils.Periodical;
import frc.robot.Utils.PeriodicalUtil;

public class Limelight implements Periodical{
    boolean tV;
        Pose2d robotPose;
        double tagDist;
        SwerveSubsystem swerve;
    public static class LimelightInputs {
        
    }

    private String tableName;
    
    public Limelight(String tableName) {
        this.tableName = tableName;

        configure(new Pose3d());

        tV = false;
        robotPose = new Pose2d();
        tagDist = 0.0;

        PeriodicalUtil.registerPeriodic(this);
    }

    public void configure(Pose3d cameraoffset) {

        double forwardOffset = cameraoffset.getX(); //in meters
        double sideOffset = cameraoffset.getY(); //in meters
        double upOffset = cameraoffset.getZ(); //in meters
        double roll = Units.radiansToDegrees(cameraoffset.getRotation().getX()); //In degrees
        double pitch = Units.radiansToDegrees(cameraoffset.getRotation().getY()); //In degrees
        double yaw = Units.radiansToDegrees(cameraoffset.getRotation().getZ()); //In degrees

        LimelightHelpers.setCameraPose_RobotSpace(
            tableName,
            forwardOffset,
            sideOffset,
            upOffset,
            roll,
            pitch,
            yaw
        );

        LimelightHelpers.setPipelineIndex(tableName, 0);
        LimelightHelpers.setLEDMode_PipelineControl(tableName);
        LimelightHelpers.setLEDMode_ForceOff(tableName);
    }
    
    public void turnOnLeds() {
        LimelightHelpers.setLEDMode_ForceOn(tableName);
    }

    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(tableName, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    
    public Pose2d getBotPose() {
        return LimelightHelpers.getBotPose2d_wpiBlue(tableName);
    }

    public LimelightHelpers.PoseEstimate getBotPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(tableName);
    }

    @Override
    public void periodic() {
        PoseEstimate poseEstimate = getBotPoseEstimate();
        if(poseEstimate == null) {
            robotPose = new Pose2d(0, 0, new Rotation2d(0));
            tagDist = 0.0;
        } else {
            robotPose = poseEstimate.pose;
            tagDist =poseEstimate.avgTagDist;
        }
        tV = LimelightHelpers.getTV(tableName);

        //System.out.println(robotPose);
    }

}
