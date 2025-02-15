// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private Timer disabledTimer;
  private AnalogInput encoder1, encoder2, encoder3, encoder4;

  public Robot() {
    m_robotContainer = new RobotContainer();
    disabledTimer =new Timer();
    encoder1 = new AnalogInput(0);
    encoder2 = new AnalogInput(1);
    encoder3 = new AnalogInput(2);
    encoder4 = new AnalogInput(3);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() 
  {
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() 
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      disabledTimer.stop();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
  public void teleopPeriodic() 
  {
    System.out.println("0: " + encoder1.getValue());
    System.out.println("1: " + encoder2.getValue());
    System.out.println("2: " + encoder3.getValue());
    System.out.println("3: " + encoder4.getValue());

  }

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
