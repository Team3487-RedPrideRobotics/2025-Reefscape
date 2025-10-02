package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax elevatorMotorLeft;
    private SparkMax elevatorMotorRight;

    private DigitalInput m_beamBreak;
    private RelativeEncoder elevatorEncoder;
    
    public ElevatorSubsystem(){
        elevatorMotorLeft = new SparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        elevatorMotorRight = new SparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
        DigitalInput m_beamBreak = new DigitalInput(0);

        elevatorEncoder = elevatorMotorLeft.getEncoder();
    }
    
    public void runMotors(double power) {
        if(!m_beamBreak.get()){
        elevatorMotorLeft.set(-power);
        elevatorMotorRight.set(power);
        }
        else{
            if(m_beamBreak.get()&&power<0){        
            elevatorMotorLeft.set(-power);
            elevatorMotorRight.set(power);}
        }
    }

    public void stopMotors()
    {
        elevatorMotorLeft.set(0);
        elevatorMotorRight.set(0);
    }

    public double getPosition()
    {
        return elevatorEncoder.getPosition();
    }
    
    public boolean elevatorPID(double goalValue,double limit, double kP, double threshold)
    {
        double delta = Math.abs(goalValue) - Math.abs(getPosition());

        System.out.println(getPosition());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            runMotors(speed);
            return false;
    }   else {
            stopMotors();
            return true;
    }

  }
}
