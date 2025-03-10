// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.OpenOption;
import java.util.function.DoubleSupplier;

import com.google.gson.internal.ObjectConstructor;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.OneShotTriggerEvent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.mutable.GenericMutableMeasureImpl;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.Camera;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.States.ElevatorManualState;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Pivot.States.PivotAnalogManual;
import frc.robot.Subsystems.Pivot.States.PivotManualState;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.FloorIntake.States.IntakePivotState;
import frc.robot.Subsystems.FloorIntake.States.IntakeState;
import frc.robot.Subsystems.FloorIntake.States.OuttakeState;

public class RobotContainer {
  public static SendableChooser<Command> autoChooser;


  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
    configureOperatorBindings();
    

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
     () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
     () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
     () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND));


    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    drivebase.getSubsystem();

    
    //Command intakePivot = new IntakePivotState(floorIntake, () -> MathUtil.applyDeadband(operatorXbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND));
    

    BuildAutoChooser();
  }

 final CommandXboxController driverXbox = new CommandXboxController(1);
 final CommandXboxController operatorXbox = new CommandXboxController(0);

 private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
 private final ArmSubsystem pivot = new ArmSubsystem();
 private final FloorIntakeSubsystem floorIntake = new FloorIntakeSubsystem();
 private final ElevatorSubsystem elevator = new ElevatorSubsystem();
 private final Camera camera = new Camera();


  private void configureBindings() {
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }
//test
  public void configureOperatorBindings()
  {

    floorIntake.setDefaultCommand(
      new IntakePivotState(
        floorIntake, 
        () -> MathUtil.applyDeadband(operatorXbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND))
      );

    pivot.setDefaultCommand(
      new PivotManualState(
        pivot, 
        () -> MathUtil.applyDeadband(operatorXbox.getLeftTriggerAxis(), OperatorConstants.LEFT_TRIGGER_DEADBAND))
      );

    operatorXbox.leftBumper().whileTrue(new PivotAnalogManual(pivot));

    elevator.setDefaultCommand(
      new ElevatorManualState(
        elevator, 
        () -> MathUtil.applyDeadband(operatorXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND))
    );

    operatorXbox.rightBumper().whileTrue(new IntakeState(floorIntake, 1));
    operatorXbox.rightTrigger(0.2).whileTrue(new OuttakeState(floorIntake, -1));
    //operatorXbox.povUp().whileTrue();

    //rightTrigger.whileTrue(new IntakeState(floorIntake, 1));
    //rightBumper.whileTrue(new IntakeState(floorIntake, -1));

    //rightJoystick.whileTrue(new IntakeState(floorIntake, 1));
    //rightJoystick.whileFalse(new IntakeState(floorIntake, -1));

    //aButton.whileTrue(new PositionState(floorIntake, "Floor"));
    //yButton.whileTrue(new PositionState(floorIntake, "Home"));
    //xButton.whileTrue(new PositionState(floorIntake, "Algae"));
    //bButton.whileTrue(new PositionState(floorIntake, "Trough"));

    /* 
    arrowUp.whileTrue(
      new ParallelCommandGroup(
        new PivotPositionState(pivot, "L4"),
        new ElevatorPositionState(elevator, "L4")
        )
    );
    arrowLeft.whileTrue(
      new ParallelCommandGroup(
        new PivotPositionState(pivot, "LMid"),
        new ElevatorPositionState(elevator, "L3")
        )
    );
    arrowRight.whileTrue(
      new ParallelCommandGroup(
        new PivotPositionState(pivot, "LMid"),
        new ElevatorPositionState(elevator, "L2")
        )
    );
    

    leftBumper.whileTrue(
      new ParallelCommandGroup(
        new PivotPositionState(pivot, "Home"),
        new ElevatorPositionState(elevator, "Intake"))
    );
    

    leftTrigger.whileFalse(new PivotPositionState(pivot, "Intake"));
    */

  }
  public Command getAutonomousCommand() {
    return null;
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

