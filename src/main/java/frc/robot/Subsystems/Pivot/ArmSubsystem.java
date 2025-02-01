package frc.robot.Subsystems.Pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
   
    private SparkMax armMotor;
    
    public ArmSubsystem()
    {
        armMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    }

    public void runMotors(double power) {
        armMotor.set(power);
    }
}

