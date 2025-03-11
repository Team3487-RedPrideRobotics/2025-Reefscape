package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

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
    
}
