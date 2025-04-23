package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.hardware.Neo;
import frc.robot.utils.hardware.NeoBuilder;
import frc.robot.utils.logging.Loggable;

public class Elevator extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ElevatorInputs {
        public double desiredElevatorHeight;
        public double setElevatorHeight;
        public double currentElevatorHeight;
        public boolean isSafe;
    }

    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private void initInputs() {
        inputs.desiredElevatorHeight = 0.0;
        inputs.setElevatorHeight = 0.0;
        inputs.currentElevatorHeight = 0.0;
        inputs.isSafe = false;
    }

    private Neo leftElevatorMotor;
    private Neo rightElevatorMotor;
    
    private boolean disableSaftey = false;

    public Elevator(){
        leftElevatorMotor = NeoBuilder.create(Constants.ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID).build();
        rightElevatorMotor = NeoBuilder.create(Constants.ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID).build();

        initInputs();
    }

    
    public void goToPos(ElevatorPosition position) {
        inputs.desiredElevatorHeight = position.getElevatorHeight();
    }

    public void goToPosNext(ElevatorPosition position) {
        nextPose = position;
    }

    public void goToNextPos() {
        goToPos(nextPose);
    }

    public boolean nextIsL4() {
        return nextPose == L4_ARMEVATOR_POSITION;
    }

    public boolean nextIsAlgae1() {
        return nextPose == ALGAE_ARMEVATOR_POSITION;
    }

    public boolean nextIsAlgae2() {
        return nextPose == ALGAE_ARMEVATOR_POSITION_TWO;
    }

    public boolean nextIsL1() {
        return nextPose == L1_ARMEVATOR_POSITION;
    }

    private void setElevtorHeight(double height){
        leftElevatorMotor.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, ELEVATOR_FF);

        inputs.setElevatorHeight = height;
    }
    
    public void setElevatorPower(double speed) {
        leftElevatorMotor.set(speed);
        leftElevatorMotor.set(speed);
    }

    public void resetElevator(double position) {
        leftElevatorMotor.getEncoder().setPosition(0);
    }

    public double getElevatorHeight() {
        return leftElevatorMotor.getEncoder().getPosition();
    }

    public void disableSaftey() {
        disableSaftey = true;
    }

    public void enableSaftey() {
        disableSaftey = false;
    }

    public boolean atDesiredPosition() {
        if(
            Math.abs(getElevatorHeight() - inputs.desiredElevatorHeight) < 0.05
        ) {
            return true;
        }

        return false;
    }

    public void safetyLogic() {
        if(disableSaftey) {
            return;
        }

        if(
            getArmRotation().getDegrees() < SAFE_ANGLE.getDegrees() - 5.0 && 
            inputs.desiredElevatorHeight <= SAFE_ELEVATOR_HEIGHT && 
            inputs.desiredElevatorHeight < getElevatorHeight() - 0.01
        ) {
            inputs.isSafe = false;
            setElevtorHeight(SAFE_ELEVATOR_HEIGHT);
            setArmRotation(SAFE_ANGLE);
            return;
        }
        
        if(!(getElevatorHeight() > Meters.convertFrom(0.1, Inches) && getElevatorHeight() < SAFE_ELEVATOR_HEIGHT)) {
            inputs.isSafe = true;
            setArmRotation(inputs.desiredArmRotation);
        } else if(inputs.desiredArmRotation.getDegrees() < SAFE_ANGLE.getDegrees()) {
            inputs.isSafe = false;
            setArmRotation(SAFE_ANGLE);
        } else {
            setArmRotation(inputs.desiredArmRotation);
            inputs.isSafe = true;
        }

        setElevtorHeight(inputs.desiredElevatorHeight);
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);

        leftElevatorMotor.log(subdirectory + "/" + humanReadableName, "leftElevatorMotor");
        rightElevatorMotor.log(subdirectory + "/" + humanReadableName, "leftElevatorMotor");
    }

    @Override
    public void periodic(){
        inputs.currentElevatorHeight = leftElevatorMotor.getEncoder().getPosition();
    
        safetyLogic();

        log("Subsystems", "Armevator");
    }
}