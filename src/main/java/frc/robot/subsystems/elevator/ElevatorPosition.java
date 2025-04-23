package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class ElevatorPosition {
    public static final ElevatorPosition HOME = new ElevatorPosition(Meters.convertFrom(0.1, Inches));



    public static class ElevatorPosition() {
        private double elevatorHeight;
        
        public ElevatorPosition(double ElevatorHeight){
            this.elevatorHeight = ElevatorHeight;
        }

        public double getElevatorHeight() {
            return elevatorHeight;
        }
    }
}
