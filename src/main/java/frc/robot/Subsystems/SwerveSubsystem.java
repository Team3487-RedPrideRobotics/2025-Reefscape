package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

import java.io.File;
import java.util.Locale.Category;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  public RobotConfig config;
  

  public SwerveSubsystem(File directory) {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e){
        e.printStackTrace();
      }
      setupPathPlanner();   
  }
      

  /**
   * Drive the robot with specified translation and rotation.
   *
   * @param translation Translation speeds (x and y).
   * @param rotation Rotation speed.
   * @param fieldRelative True if the input is field-relative, false if robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Drive the robot with ChassisSpeeds (speed and rotation).
   *
   * @param velocity ChassisSpeeds object containing speed and rotation information.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void driveRobotRelative(ChassisSpeeds e)
  {
    swerveDrive.drive(e);
  }

  /**
   * Reset odometry to a specific pose.
   *
   * @param pose The pose to reset odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  /**
   * Get the robot's current pose.
   *
   * @return The robot's Pose2d.
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Zero the robot's gyro angle and reset odometry to face forward.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Get the current heading of the robot.
   *
   * @return The robot's heading as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Command to drive the robot based on joystick inputs for translation and rotation.
   *
   * @param translationX Joystick input for X translation.
   * @param translationY Joystick input for Y translation.
   * @param rotation Joystick input for rotation.
   * @return Command to drive the robot.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(() -> {
      Translation2d translation = new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
      );
      double rot = rotation.getAsDouble() * Math.PI;
      driveFieldOriented(new ChassisSpeeds(translation.getX(), translation.getY(), rot));
    });
  }

  @Override
  public void periodic() {
    // Periodic code can be added here
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void setupPathPlanner()
  {
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry, 
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    Constants.AutonConstants.TRANSLATION_PID, // Translation PID constants
                    Constants.AutonConstants.ANGLE_PID // Rotation PID constants
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
  } 
  


  public void lock()
  {
    swerveDrive.lockPose();
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }
}