package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends Command {

    private SparkMax elevatorMotorLeft;
    private SparkMax elevatorMotorRight;

    private RelativeEncoder elevatorEncoder;
    
    public ElevatorSubsystem(){
        elevatorMotorLeft = new SparkMax(Constants.ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        elevatorMotorRight = new SparkMax(Constants.ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        elevatorEncoder = elevatorMotorLeft.getEncoder();
    }
    
    public void runMotors(double power) {
        elevatorMotorLeft.set(-power);
        elevatorMotorRight.set(power);
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

    public boolean goToAngle(double elevator, double limit, double elevatorkP, double threshold){
        double armDelta = Math.abs(elevator) - Math.abs(getPosition());
        System.out.println("elevator difference: " + armDelta);
        System.out.println("elevator position: " + getPosition());

        if(Math.abs(armDelta) >= threshold){
            var motorSpeed = -armDelta*elevatorkP;
            motorSpeed = Math.abs(motorSpeed) > limit ? limit * Math.signum(motorSpeed) : motorSpeed;
            System.out.println("motor speed: " + motorSpeed);
            runMotors(motorSpeed);
            return false;
          } else {
            stopMotors();
            return true;
          }

    
      }
    
}
