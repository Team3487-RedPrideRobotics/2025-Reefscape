// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.google.gson.internal.ObjectConstructor;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.OneShotTriggerEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.FloorIntake.States.IntakeState;
import frc.robot.Subsystems.FloorIntake.States.OuttakeState;
import frc.robot.Subsystems.FloorIntake.States.PositionState;

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
 private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();


  private void configureBindings() {

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  

    
  }

  public void configureOperatorBindings()
  {
    Trigger rightTrigger  = operatorXbox.rightTrigger(0.2);
    Trigger rightBumper   = operatorXbox.rightBumper();
    Trigger leftTrigger   = operatorXbox.leftTrigger(0.2);
    Trigger leftBumper    = operatorXbox.leftBumper();
    Trigger rightJoystick = operatorXbox.rightStick();
    Trigger xButton       = operatorXbox.x();
    Trigger yButton       = operatorXbox.y();
    Trigger aButton       = operatorXbox.a();
    Trigger bButton       = operatorXbox.b();

    rightJoystick.whileTrue(new IntakeState(floorIntake, 1));
    rightJoystick.whileFalse(new IntakeState(floorIntake, -1));

    aButton.whileTrue(new PositionState(floorIntake, "Floor"));
    yButton.whileTrue(new PositionState(floorIntake, "Home"));
    xButton.whileTrue(new PositionState(floorIntake, "Algae"));
    bButton.whileTrue(new PositionState(floorIntake, "Trough"));

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
