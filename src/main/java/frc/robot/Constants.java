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
    
    public static final double ROBOT_MASS = (50) * 0.453592; // 50lbs (50 lbs exactly ikr what a thing) * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms spark max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(3);
   
    public static class OperatorConstants
    {

        // Joystick Deadband
        public static final double LEFT_X_DEADBAND  = 0.1;
        public static final double LEFT_Y_DEADBAND  = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT    = 6;
    }


    public static final class AutonConstants
    {

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
        public static final PIDConstants ANGLE_PID       = new PIDConstants(5, 0, 0.01);
        public static final double AUTO_MAX_SPEED = 2.57; //  meters per second
    }

    
    public static final class DrivebaseConstants
    {
  
      // Hold time on motor brakes when disabled
      public static final double WHEEL_LOCK_TIME = 5; // seconds
    }

    public static final class ArmConstants
    {
      public static final int ARM_MOTOR_ID = 99;
    }
    
    
}