package frc.robot.subsystems.intake;

public class floorIntake extends SubsystemBase{
    private SparkMax floorIntakeMotor1;
    private SparkMax floorIntakeMotor2;

    public void runMotor(double power1, double power2) {
        floorIntakeMotor1.set(power1);
        floorIntakeMotor2.set(power2);
    }
}

