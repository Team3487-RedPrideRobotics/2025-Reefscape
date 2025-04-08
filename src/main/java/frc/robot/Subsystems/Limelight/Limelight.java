package frc.robot.Subsystems.Limelight;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Utils.Loggable;
import frc.robot.Utils.Periodical;
import frc.robot.Utils.PeriodicalUtil;

public class Limelight implements Periodical, Loggable{
    @AutoLog
    public static class LimelightInputs {
        boolean tV;
        Pose2d robotPose;
        double tagDist;
    }

    
    LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();

    private String tableName;
    
    public Limelight(String tableName) {
        this.tableName = tableName;

        configure(new Pose3d());

        inputs.tV = false;
        inputs.robotPose = new Pose2d();
        inputs.tagDist = 0.0;

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
        LimelightHelpers.PoseEstimate poseEstimate = getBotPoseEstimate();
        if(poseEstimate == null) {
            inputs.robotPose = new Pose2d(0, 0, new Rotation2d(0));
            inputs.tagDist = 0.0;
        } else {
            inputs.robotPose = poseEstimate.pose;
            inputs.tagDist = poseEstimate.avgTagDist;
        }
        inputs.tV = LimelightHelpers.getTV(tableName);

        log("Limelights", tableName);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
