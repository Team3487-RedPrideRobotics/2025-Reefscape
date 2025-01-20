// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  public static SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
     () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));


    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    drivebase.getSubsystem();

    BuildAutoChooser();
  }

 final CommandXboxController driverXbox = new CommandXboxController(1);
 final CommandXboxController operatorXbox = new CommandXboxController(0);

 private final SwerveSubsystem drivebase = 
 new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
 

  private void configureBindings() {

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  
  }


  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("8 Auto");

    //try{
      //return autoChooser.getSelected();
    //} catch(Exception e ) {
      //throw e;
    //}  
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
  public static void BuildAutoChooser(){
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
}
