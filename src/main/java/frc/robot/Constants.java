package frc.robot;


import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants {

    RobotConfig config;{
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
}
    
    public static final double ROBOT_MASS = (104) * 0.453592; // kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms spark max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(15);
   
    public static class DriverConstants
    {
        public static final double LEFT_X_DEADBAND  = 0.5;
        public static final double LEFT_Y_DEADBAND  = 0.5;
        public static final double RIGHT_X_DEADBAND = 0.5;
        public static final double TURN_CONSTANT    = 6;
    }

    public static class OperatorConstants
    {
      
      public static final double RIGHT_Y_DEADBAND = 0.2;
      public static final double LEFT_Y_DEADBAND = 0.2;

      public static final double LEFT_TRIGGER_DEADBAND = 0.2;
    }


    public static final class AutonConstants
    {

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.5, 0, 0);
        public static final PIDConstants ANGLE_PID       = new PIDConstants(0, 0, 0.01);
        public static final double AUTO_MAX_SPEED = 2.57; //  meters per second
    }

    
    public static final class DrivebaseConstants
    {
  
      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 5; // seconds
    }
  
    public static final class ElevatorConstants
    {
      public static final int LEFT_MOTOR_ID  = 43;
      public static final int RIGHT_MOTOR_ID = 44;
    }
    public static final class ArmConstants
    {
      public static final int ARM_MOTOR_ID = 40;

      public static final double PIVOT_SPEED = 0.2;
    }
    public static final class FloorConstants
    {
      public static final int FLOOR_INTAKE_MOTOR_ID = 41;
      public static final int FLOOR_PIVOT_MOTOR_ID = 42;

      public static final double FlOOR_PIVOT_SPEED = 0.3;


    }
    
    
}
