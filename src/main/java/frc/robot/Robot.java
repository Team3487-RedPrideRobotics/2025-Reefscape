// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.LightSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Timer disabledTimer;

  private LightSubsystem m_lightSubsystem;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    disabledTimer =new Timer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() 
  {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.runPattern(m_lightSubsystem.m_rainbow);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() 
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    
    // Start Intitializing LEDs
     m_led = new AddressableLED(9);
     m_ledBuffer = new AddressableLEDBuffer(60);
     m_led.setLength(m_ledBuffer.getLength());
     m_led.setData(m_ledBuffer);
     m_led.start();
     // End Initializing LEDs
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

    var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Red) {
          m_robotContainer.runPattern(m_lightSubsystem.red);
        } else {
          m_robotContainer.runPattern(m_lightSubsystem.blue);
        }
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
}
