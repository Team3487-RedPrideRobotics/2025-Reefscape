package frc.robot.Subsystems.FloorIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntakeSubsystem extends SubsystemBase{
    private SparkMax floorIntakeMotor, floorPivotMotor;
    private RelativeEncoder pivotEncoder;

    public FloorIntakeSubsystem()
    {
        floorIntakeMotor = new SparkMax(Constants.FloorConstants.FLOOR_INTAKE_MOTOR_ID, MotorType.kBrushless);
        floorPivotMotor  = new SparkMax(Constants.FloorConstants.FLOOR_PIVOT_MOTOR_ID, MotorType.kBrushless);

        pivotEncoder = floorPivotMotor.getEncoder();
    }

    public void runFloorIntake(double balkanRage)
    {
        floorIntakeMotor.set(balkanRage);
    }

    public void runFloorPivot(double balkanRage)
    {
        floorPivotMotor.set(balkanRage);
    }

    public void stopFloorIntake()
    {
        floorIntakeMotor.set(0);
    }

    public void stopFloorPivot()
    {
        floorPivotMotor.set(0);
    }

    public double getPivotValue(){
        return pivotEncoder.getPosition();
    }
    
    
    public boolean FloorPID(double goalValue,double limit, double kP, double threshold)
    {
        double delta = Math.abs(goalValue) - Math.abs(getPivotValue());

        System.out.println(getPivotValue());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            runFloorPivot(speed);
            System.out.println("turn up that skibidi RIZZ");
            return false;
    }   else {
            System.out.println("Erm what the sigma");
            stopFloorPivot();
            return true;
    }

  }
    

}
