
package frc.robot.Subsystems.FloorIntake.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.FloorIntake.FloorIntakeSubsystem;

public class SlowOuttakeState extends Command
    {
    private double intakeSpeed;
    private FloorIntakeSubsystem subsystem;

    public SlowOuttakeState(FloorIntakeSubsystem skibidi)
    {
        super();
        subsystem = skibidi;
    }

    public void initialize(){
        subsystem.runFloorIntake(-0.5);
    }

    public void end(boolean interrupted)
    {
        subsystem.stopFloorIntake();
    }
    
}