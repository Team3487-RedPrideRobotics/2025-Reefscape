
package frc.robot.Subsystems.FloorIntake.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;

public class IntakeState extends Command
{
    private double intakeSpeed;
    private FloorIntakeSubsystem subsystem;

    public IntakeState(FloorIntakeSubsystem skibidi, double balkanRage)
    {
        super();
        this.intakeSpeed = balkanRage;
        subsystem = skibidi;
    }

    public void initialize(){
        subsystem.runFloorIntake(intakeSpeed);
    }

    public void end(boolean interrupted)
    {
        subsystem.stopFloorIntake();
    }
    
}