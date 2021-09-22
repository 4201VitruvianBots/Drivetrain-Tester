/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.TalonFXCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TalonFXDriveTrain;

import java.util.function.DoubleSupplier;

/**
 * An example command that uses an example subsystem.
 */
public class TaloxFXSetTankDrive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final TalonFXDriveTrain m_driveTrain;
    private final DoubleSupplier m_leftOutput, m_rightOutput;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TaloxFXSetTankDrive(TalonFXDriveTrain subsystem, DoubleSupplier leftOutput, DoubleSupplier rightOutput) {
        m_driveTrain = subsystem;
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.setMotorTankDrive(m_leftOutput.getAsDouble(), m_rightOutput.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
