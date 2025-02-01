package frc.robot.Subsystems.FloorIntake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class FloorIntakeSubsystem {
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
        floorIntakeMotor.set(balkanRage);
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

    public boolean goToAngle(double pivot, double limit, double pivotkP, double threshold){
        double pivotDelta = Math.abs(pivot) - Math.abs(getPivotValue());
        System.out.println("pivot difference: " + pivotDelta);
        System.out.println("pivot position: " + getPivotValue());

        if(Math.abs(pivotDelta) >= threshold){
            var motorSpeed = -pivotDelta*pivotkP;
            motorSpeed = Math.abs(motorSpeed) > limit ? limit * Math.signum(motorSpeed) : motorSpeed;
            System.out.println("motor speed: " + motorSpeed);
            runFloorPivot(motorSpeed);
            return false;
          } else {
            stopFloorPivot();
            return true;
          }

    
      }

}
