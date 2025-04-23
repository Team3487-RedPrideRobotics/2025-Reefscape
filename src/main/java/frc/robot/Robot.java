
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.commandUtils.PeriodicalUtil;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

    Logger.recordMetadata("RobotName", "2025Bot"); // Set a metadata value

    try {
      if(isReal()) {
        Logger.addDataReceiver(new WPILOGWriter("/U/Logs")); // Log to a USB stick ("/U/logs")
        SignalLogger.setPath("/U/Logs");
        SignalLogger.stop();
      }
    } catch (Exception e) {
      DriverStation.reportWarning(e.getMessage(), e.getStackTrace());

      Logger.addDataReceiver(new WPILOGWriter("/logs"));
      SignalLogger.setPath("/logs");
      SignalLogger.stop();
    }
    
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      
    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, r eplay sources, or metadata values may
                    // be added.

    RobotContainer.registerNamedCommands();
    m_robotContainer = new RobotContainer();

    PathfindingCommand.warmupCommand().schedule();
    
    RobotController.setBrownoutVoltage(6.25);
  }

  @Override
  public void robotPeriodic() {
    PeriodicalUtil.runPeriodics();

    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
