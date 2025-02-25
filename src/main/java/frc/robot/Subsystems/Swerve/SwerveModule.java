// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private final PIDController m_turningPidController;

  private final AnalogInput m_absoluteEncoder;
  private final boolean m_absoluteEncoderReversed;
  private final double m_absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed ) {
            this.m_absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.m_absoluteEncoderReversed = absoluteEncoderReversed;
            m_absoluteEncoder = new AnalogInput(absoluteEncoderId);

            m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
            m_turnMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

            m_driveEncoder = m_driveMotor.getEncoder();
            m_turnEncoder = m_turnMotor.getEncoder();

            m_turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
            m_turningPidController.enableContinuousInput(-Math.PI, Math.PI);

            

            resetEncoders();
            
    }

    
    public double getDrivePosition() {
        return m_driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return m_turnEncoder.getPosition() * ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return m_driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return m_turnEncoder.getVelocity() * ModuleConstants.kTurningEncoderRPM2RadPerSec;
    }

    
    public double getAbsoluteEncoderRad() {
        double angle = m_absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= m_absoluteEncoderOffsetRad;
        return angle * (m_absoluteEncoderReversed ? -1.0 : 1.0);
    }

       public void resetEncoders() {
        m_driveEncoder.setPosition(0);
        m_turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        m_driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        m_turnMotor.set(m_turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + m_absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        m_driveMotor.set(0);
        m_turnMotor.set(0);
    }

}