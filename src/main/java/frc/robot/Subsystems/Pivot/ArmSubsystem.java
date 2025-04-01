package frc.robot.Subsystems.Pivot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
   
    private SparkMax armMotor, shootMotor;
    private RelativeEncoder armEncoder;
    
    public ArmSubsystem()
    {
        armMotor = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
        shootMotor = new SparkMax(Constants.ArmConstants.SHOOT_MOTOR_ID, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
    }

    public void setPivotMotor(double power) {
        armMotor.set(power);
    }

    public void stopPivotMotor()
    {
        armMotor.set(0);
    }

    public void setShootMotor(double power){
        shootMotor.set(power);
    }

    public void stopShootMotor()
    {
        shootMotor.set(0);
    }

    public double getValue()
    {
        return armEncoder.getPosition();
    }   

    public void pidMotor(double Power){
        armMotor.set(-Power);
    }
    
    public boolean pivotPID(double goalValue,double limit, double kP, double threshold){
    
        double delta = Math.abs(goalValue) - Math.abs(getValue());

        System.out.println(getValue());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            pidMotor(speed);
            System.out.println("turn up that skibidi RIZZ");
            return false;
    }   else {
            System.out.println("Erm what the sigma");
            stopPivotMotor();
            return true;
            
    }

    }
}

