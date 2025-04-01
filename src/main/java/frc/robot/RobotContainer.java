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
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Elevator.States.ElevatorManualState;
import frc.robot.Subsystems.Pivot.ArmSubsystem;
import frc.robot.Subsystems.Pivot.States.PivotAnalogManual;
import frc.robot.Subsystems.Pivot.States.PivotManualState;
<<<<<<< Updated upstream
=======
import frc.robot.Subsystems.Pivot.States.PivotZeroState;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Subsystems.Swerve.States.ForwardState;
import frc.robot.Subsystems.Swerve.States.LeftState;
import frc.robot.Subsystems.Swerve.States.LimelightForwardState;
import frc.robot.Subsystems.Swerve.States.LimelightLeftState;
import frc.robot.Subsystems.Swerve.States.LimelightRightState;
import frc.robot.Subsystems.Swerve.States.RightState;
>>>>>>> Stashed changes
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;
import frc.robot.Subsystems.FloorIntake.States.IntakePivotState;
import frc.robot.Subsystems.FloorIntake.States.IntakeState;
import frc.robot.Subsystems.FloorIntake.States.OuttakeState;
import frc.robot.Subsystems.FloorIntake.States.SlowOuttakeState;

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
    //I dont think camera is needed with pi stuff, but scared to remove
    camera = new Camera();
   
    configureDriverBindings();
    configureOperatorBindings();
    
    buildNamedCommands();
    BuildAutoChooser();
  }


  private void configureDriverBindings() {
<<<<<<< Updated upstream
    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
=======
    //
    // Creating Double Supplier for elevator sensor, allowing slowing down while elevator high
    //
    DoubleSupplier elevatorValue = (() -> elevator.getPosition());

    //
    // Driving field oriented
    //
    Command FieldOriented = drivebase.driveCommand(
>>>>>>> Stashed changes
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
     true);

    //
    // Driving front of robot centric
    //
    Command FrontCentric = drivebase.driveCommand( 
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
          false);

    //
    // Driving back of robot centric
    //
    Command BackCentric = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), DriverConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), DriverConstants.LEFT_X_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getRightX(), DriverConstants.RIGHT_X_DEADBAND),
          false);

    drivebase.setDefaultCommand(FieldOriented);
    drivebase.getSubsystem();
    
    //
    // A Button: Reset Gyro
    // X Button: Lock/X wheels
    // Y Button: Toggle FloorIntake Up
    // 
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverXbox.y().toggleOnTrue(new FloorIntakeZeroState(floorIntake));

<<<<<<< Updated upstream
    driverXbox.leftBumper().whileTrue(AHHHH_WHY_HAVE_YOU_FORSAKEN_ME);
    driverXbox.rightBumper().whileTrue(floorMan);
=======
    
    //
    // Left Bumper: Activate front centric
    //
    driverXbox.leftBumper().whileTrue(FrontCentric);

    //
    // Right Bumbper: Activate back centric  
    //
    driverXbox.rightBumper().whileTrue(BackCentric);

    //
    // Up Arrow: Limelight Allign "Forward"
    // Left Arrow: Limelight Allign Left
    // Right Arrow: Limelight Allign Right
    //
    driverXbox.povUp().whileTrue(new LimelightForwardState(drivebase));
    driverXbox.povLeft().whileTrue(new LimelightLeftState(drivebase));
    driverXbox.povRight().whileTrue(new LimelightRightState(drivebase));

    //
    // Left Trigger: Left
    // Right Trigger: Right
    //
    driverXbox.leftTrigger().whileTrue(new LeftState(drivebase));
    driverXbox.rightTrigger().whileTrue(new RightState(drivebase));


>>>>>>> Stashed changes
  }

  public void configureOperatorBindings()
  {

    //
    // Right Joystick Up and Down, effects the floorIntake's pivot
    //
    floorIntake.setDefaultCommand(
      new IntakePivotState(
        floorIntake, 
        () -> MathUtil.applyDeadband(operatorXbox.getRightY(), OperatorConstants.RIGHT_Y_DEADBAND))
      );

    //
    // Left Trigger turns the pivot "out"
    //
    pivot.setDefaultCommand(
      new PivotManualState(
        pivot, 
        () -> MathUtil.applyDeadband(operatorXbox.getLeftTriggerAxis(), OperatorConstants.LEFT_TRIGGER_DEADBAND))
      );

    //
    // Left Bumper turns the pivot "in"
    //
    operatorXbox.leftBumper().whileTrue(new PivotAnalogManual(pivot));

    //
    // Left Joystick Up and Down, effects the elevator
    //
    elevator.setDefaultCommand(
      new ElevatorManualState(
        elevator, 
        () -> MathUtil.applyDeadband(operatorXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND))
    );

<<<<<<< Updated upstream
    operatorXbox.rightBumper().whileTrue(new IntakeState(floorIntake, 1));
    operatorXbox.rightTrigger(0.2).whileTrue(new OuttakeState(floorIntake, 1));
=======
    //
    // Right Bumper intakes both pivot and floor intake
    //
    operatorXbox.rightBumper().whileTrue(new ParallelCommandGroup(
      new IntakeState(floorIntake, 1),
      new ArmSuckState(pivot)
    ));

    //
    // Right Trigger outtakes both pivot and floor intake
    //
    operatorXbox.rightTrigger(0.2).whileTrue( 
      new ParallelCommandGroup(new ParallelCommandGroup(
        new OuttakeState(floorIntake, 1),
        new ArmShootState(pivot)
        )));

    //
    // Y button: Elevator L4
    // B button: Elevator L3
    // X button: Elevator L2
    // A Button: Elevator Home
    //
    operatorXbox.y().whileTrue(new ElevatorPermL4State(elevator));
    operatorXbox.b().whileTrue(new ElevatorPermL3State(elevator));
    operatorXbox.x().whileTrue(new ElevatorPermL2State(elevator));
    operatorXbox.a().whileTrue(new ElevatorBaseState(elevator));
    
    //
    // Up arrow moves pivot to an angle to score L4
    //
    operatorXbox.povUp().whileTrue(new PivotL4ScoredState(pivot));

    //
    // Down arrow moves elevator to intake position, moves pivot to intake angle, and intakes pivot
    //
    operatorXbox.povDown().whileTrue(
      new ParallelCommandGroup(
          new PivotIntakeState(pivot), 
          new ElevatorIntakeState(elevator)));
>>>>>>> Stashed changes


  }
 
  public Command getAutonomousCommand() {
<<<<<<< Updated upstream
    //return autoChooser.getSelected();
    return new PathPlannerAuto("A-P4_T10;");
  }

  public void buildNamedCommands(){
    NamedCommands.registerCommand("Floor Intake Shoot", new SlowOuttakeState(floorIntake));
=======
    // returns what the AutoChooser selected
    return autoChooser.getSelected();
  }

  public void buildNamedCommands(){

    //
    // Floor Intake Angle
    //
    NamedCommands.registerCommand("Floor Pivot Trough", new TroughState(floorIntake));
    NamedCommands.registerCommand("Floor Pivot Home", new FloorIntakeZeroState(floorIntake));
    NamedCommands.registerCommand("Floor Intake Shoot", new OuttakeState(floorIntake, 1).withTimeout(2));

    //
    // Elevator
    //
    NamedCommands.registerCommand("Elevator L4", new ElevatorL4State(elevator));

    //
    // Pivot
    //
    NamedCommands.registerCommand("Pivot L4", new PivotL4ScoredState(pivot));
    NamedCommands.registerCommand("Pivot Zero", new PivotZeroState(pivot));
    NamedCommands.registerCommand("Arm Shoot", new ArmShootState(pivot).withTimeout(1));
    NamedCommands.registerCommand("Claw Intake",   new ParallelCommandGroup(new PivotIntakeState(pivot), new ElevatorIntakeState(elevator)));

    //
    // Driving
    //
    NamedCommands.registerCommand("Limelight Left", new LimelightLeftState(drivebase));
    NamedCommands.registerCommand("Limelight Forward", new LimelightForwardState(drivebase));
    NamedCommands.registerCommand("Forward 0.5", new ForwardState(drivebase).withTimeout(0.5));
    NamedCommands.registerCommand("Forward 1.5", new ForwardState(drivebase).withTimeout(1.5));
>>>>>>> Stashed changes
  }

  public void setMotorBrake(boolean brake)
  {
    //Sets drive motors to brake mode when given true value
    drivebase.setMotorBrake(brake);
  }
  
  public static void BuildAutoChooser(){
    //Creates the auto chooser, and then adds it to shuffleboard
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

}

