package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class DisableClimbMode extends SequentialCommandGroup {
    public DisableClimbMode(Climber climber) {
        addCommands(
                    new SetClimbMode(climber, false),
                    new RetractClimber(climber));
    }

}
