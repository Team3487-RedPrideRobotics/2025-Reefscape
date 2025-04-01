// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.OpenOption;
import java.util.function.DoubleSupplier;

import com.google.gson.internal.ObjectConstructor;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
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
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.States.ElevatorBaseState;
import frc.robot.Subsystems.Elevator.States.ElevatorIntakeState;
import frc.robot.Subsystems.Elevator.States.ElevatorL4State;
import frc.robot.Subsystems.Elevator.States.ElevatorManualState;
import frc.robot.Subsystems.Elevator.States.ElevatorPermL2State;
import frc.robot.Subsystems.Elevator.States.ElevatorPermL3State;
import frc.robot.Subsystems.Elevator.States.ElevatorPermL4State;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Pivot.States.ArmShootState;
import frc.robot.Subsystems.Pivot.States.ArmSuckState;
import frc.robot.Subsystems.Pivot.States.PivotAnalogManual;
import frc.robot.Subsystems.Pivot.States.PivotIntakeState;
import frc.robot.Subsystems.Pivot.States.PivotL4ScoredState;
import frc.robot.Subsystems.Pivot.States.PivotManualState;
import frc.robot.Subsystems.Pivot.States.PivotZeroState;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Subsystems.Swerve.States.BackwardTimedState;
import frc.robot.Subsystems.Swerve.States.ForwardState;
import frc.robot.Subsystems.Swerve.States.LeftState;
import frc.robot.Subsystems.Swerve.States.LimelightForwardState;
import frc.robot.Subsystems.Swerve.States.LimelightLeftState;
import frc.robot.Subsystems.Swerve.States.LimelightRightState;
import frc.robot.Subsystems.Swerve.States.RightState;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.FloorIntake.States.FloorIntakeZeroState;
import frc.robot.Subsystems.FloorIntake.States.IntakePivotState;
import frc.robot.Subsystems.FloorIntake.States.IntakeState;
import frc.robot.Subsystems.FloorIntake.States.OuttakeState;
import frc.robot.Subsystems.FloorIntake.States.SlowOuttakeState;
import frc.robot.Subsystems.FloorIntake.States.TroughState;

public class RobotContainer {
  public static SendableChooser<Command> autoChooser;
  
  private final SwerveSubsystem drivebase;
  private final ArmSubsystem pivot;
  private final FloorIntakeSubsystem floorIntake;
  private final ElevatorSubsystem elevator;
  private final Camera camera;

  private final CommandXboxController driverXbox;
  final CommandXboxController operatorXbox;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    driverXbox = new CommandXboxController(1);
    operatorXbox = new CommandXboxController(0);
   
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    pivot = new ArmSubsystem();
    floorIntake = new FloorIntakeSubsystem();
    elevator = new ElevatorSubsystem();
    camera = new Camera();
   
    configureDriverBindings();
    configureOperatorBindings();
    
    buildNamedCommands();
    BuildAutoChooser();
  }


  private void configureDriverBindings() {
    DoubleSupplier elevatorValue = (() -> elevator.getPosition());
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
      true,
      elevatorValue);

    Command AHHHH_WHY_HAVE_YOU_FORSAKEN_ME = drivebase.driveCommand( 
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
      false,
      elevatorValue);

    Command floorMan = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
      false,
      elevatorValue);

            
    

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    drivebase.getSubsystem();
    
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    driverXbox.leftBumper().whileTrue(AHHHH_WHY_HAVE_YOU_FORSAKEN_ME);
    driverXbox.rightBumper().whileTrue(floorMan);


    driverXbox.povLeft().whileTrue(new LimelightLeftState(drivebase));
    driverXbox.povRight().whileTrue(new LimelightRightState(drivebase));
    driverXbox.povUp().whileTrue(new LimelightForwardState(drivebase));
    driverXbox.povDown().whileTrue(new ForwardState(drivebase));


    driverXbox.leftTrigger().whileTrue(new LeftState(drivebase));
    driverXbox.rightTrigger().whileTrue(new RightState(drivebase));

    driverXbox.y().toggleOnTrue(new FloorIntakeZeroState(floorIntake));

  }

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

    operatorXbox.rightBumper().whileTrue(new ParallelCommandGroup(
      new IntakeState(floorIntake, 1),
      new ArmSuckState(pivot)
    ));
    operatorXbox.rightTrigger(0.2).whileTrue( 
      new ParallelCommandGroup(new ParallelCommandGroup(
        new OuttakeState(floorIntake, 1),
        new ArmShootState(pivot)
        )));

    operatorXbox.y().whileTrue(new ElevatorPermL4State(elevator));
    operatorXbox.x().whileTrue(new ElevatorPermL2State(elevator));
    operatorXbox.b().whileTrue(new ElevatorPermL3State(elevator));
    operatorXbox.a().whileTrue(new ElevatorBaseState(elevator));
    
    operatorXbox.povUp().whileTrue(new PivotL4ScoredState(pivot));
    operatorXbox.povDown().whileTrue(
      new ParallelCommandGroup(
          new PivotIntakeState(pivot), 
          new ElevatorIntakeState(elevator)));


  }
 

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void buildNamedCommands(){
    NamedCommands.registerCommand("Floor Pivot Trough", new TroughState(floorIntake));
    NamedCommands.registerCommand("Floor Intake Shoot", new OuttakeState(floorIntake, 1).withTimeout(2));
    NamedCommands.registerCommand("Floor Pivot Home", new FloorIntakeZeroState(floorIntake));
    NamedCommands.registerCommand("Elevator L4", new ElevatorL4State(elevator));
    NamedCommands.registerCommand("Pivot L4", new PivotL4ScoredState(pivot));
    NamedCommands.registerCommand("Arm Shoot", new ArmShootState(pivot).withTimeout(1));
    NamedCommands.registerCommand("Limelight Left", new LimelightLeftState(drivebase));
    NamedCommands.registerCommand("Limelight Forward", new LimelightForwardState(drivebase));
    NamedCommands.registerCommand("Forward 1.5", new ForwardState(drivebase).withTimeout(1.5));
    NamedCommands.registerCommand("Forward 0.5", new ForwardState(drivebase).withTimeout(0.5));
    NamedCommands.registerCommand("Pivot Zero", new PivotZeroState(pivot));
    NamedCommands.registerCommand("Claw Intake",   new ParallelCommandGroup(new PivotIntakeState(pivot), new ElevatorIntakeState(elevator)));
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

