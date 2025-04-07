package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class State<T extends Subsystem> extends Command {
    protected T requiredSubsystem;

    public State(T requiredSubsystem) {
        this.requiredSubsystem = requiredSubsystem;

        addRequirements(requiredSubsystem);
    }
}