package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
   
    private SparkMax armMotor;
    
    public void runMotors(double Power) {
        armMotor.set(Power);
    }
}

