package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;


public class Vision extends SubsystemBase{ 
    NetworkTable table;

    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;

    public Vision(){
        table = NetworkTableInstance.getDefault().getTable("Limelight");

        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");

    }

    public double getXPos(){
        return tx.getDouble(0.0);
    }

    public double getYPos(){
        return ty.getDouble(0.0);
    }

    public double getArea(){
       return ta.getDouble(0.0);
    }

    public double getID(){
        return tid.getDouble(0);
    }

    public double getDistance(){
        
        double distance = 0.0;

        double targetOffsetAngle_Vertical = Rotation2d.fromDegrees(Constants.VisionConstants.limelightMountAngleDegrees + ty.getDouble(0.0)).getRadians();

        distance = (Constants.VisionConstants.heightOfGoal-Constants.VisionConstants.limelightLensHeightInches) / Math.tan(targetOffsetAngle_Vertical);
  
        return distance;
      }
    

    public void Periodic(){
        System.out.println("Hi");

        SmartDashboard.putNumber("XPos", getXPos());
        SmartDashboard.putNumber("YPos", getYPos());
        SmartDashboard.putNumber("Area", getArea());
        SmartDashboard.putNumber("April Tag", getID());
        SmartDashboard.putNumber("Distance", getDistance());
    }
    


}