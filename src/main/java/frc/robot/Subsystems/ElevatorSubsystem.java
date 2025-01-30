package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax elevatorMotor1;
    private SparkMax elevatorMotor2;
    
    public ElevatorSubsystem(){
    }
    
    public void runMotors(double power1) {
        elevatorMotor1.set(power1);
        elevatorMotor2.set(power1);
    }

    
}
