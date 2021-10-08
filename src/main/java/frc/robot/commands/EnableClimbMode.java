package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

public class EnableClimbMode extends SequentialCommandGroup {
    public EnableClimbMode(Climber climber) {
        addCommands(
                    new SetClimbMode(climber, true),
                    new ExtendClimber(climber));
    }

}
