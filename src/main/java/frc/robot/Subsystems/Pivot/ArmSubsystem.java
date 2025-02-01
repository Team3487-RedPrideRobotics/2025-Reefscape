package frc.robot.Subsystems.Pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
   
    private SparkMax armMotor;
    private RelativeEncoder armEncoder;
    
    public ArmSubsystem()
    {
        armMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
    }

    public void setMotor(double power) {
        armMotor.set(power);
    }

    public void stopMotor()
    {
        armMotor.set(0);
    }

    public double getValue()
    {
        return armEncoder.getPosition();
    }

    public boolean goToAngle(double arm, double limit, double armkP, double threshold){
        double armDelta = Math.abs(arm) - Math.abs(getValue());
        System.out.println("arm difference: " + armDelta);
        System.out.println("arm position: " + getValue());

        if(Math.abs(armDelta) >= threshold){
            var motorSpeed = -armDelta*armkP;
            motorSpeed = Math.abs(motorSpeed) > limit ? limit * Math.signum(motorSpeed) : motorSpeed;
            System.out.println("motor speed: " + motorSpeed);
            setMotor(motorSpeed);
            return false;
          } else {
            stopMotor();
            return true;
          }

    
      }
}

