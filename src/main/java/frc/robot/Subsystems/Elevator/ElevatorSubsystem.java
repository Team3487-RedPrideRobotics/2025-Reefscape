package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax elevatorMotor1, elevatorMotor2;
    
    public ElevatorSubsystem()
    {
        elevatorMotor1 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID_ONE, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_MOTOR_ID_TWO, MotorType.kBrushless);
    }
    
    public void runMotors(double power) {
        elevatorMotor1.set(-power);
        elevatorMotor2.set(power);
    }

    
}
