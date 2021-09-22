/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.TalonFXCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonFXDriveTrain;

/**
 * An example command that uses an example subsystem.
 */
public class TalonFXBrakeWhileHeld extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final TalonFXDriveTrain m_driveTrain;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TalonFXBrakeWhileHeld(TalonFXDriveTrain subsystem) {
        m_driveTrain = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.setDriveTrainNeutralMode(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.setDriveTrainNeutralMode(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
