/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Intake;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeIntake extends CommandBase {
    private final Intake m_intake;

    public IntakeIntake(Intake intake) {
        m_intake = intake;

        // Use requires() here to declare subsystem dependencies
        addRequirements(intake);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        switch (Intake.intakeState) {
            case 2:
            case 1:
            case 0:
            default:
                //m_intake.setHarpoonExtend(false);
                break;
        }

        Intake.overridePassive = true;
    }
    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        switch (Intake.intakeState) {
            case 2:
                m_intake.setCargoIntakeOutput(Constants.CARGO_INTAKE_SPEED);
                break;
            case 1:
                m_intake.setHatchGroundIntakeOutput(Constants.HATCH_GROUND_INTAKE_SPEED);
                break;
            case 0:
            default:
                //m_intake.setHarpoonExtend(true);
                m_intake.setHatchIntakeOutput(Constants.HATCH_INTAKE_SPEED);
                //m_intake.setHarpoonSecure(false);
                //Timer.delay(0.2);
                //m_intake.setHarpoonSecure(true);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        if(Intake.intakeState == 2)
            return m_intake.bannerIR.get();
        else
            return false;
    }
    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        switch (Intake.intakeState) {
            case 2:
//                if (m_intake.bannerIR.get()) {
//                    Timer.delay(0.25);
//                    Robot.wrist.setAbsolutePosition(Constants.WRIST_RETRACTED_CARGO_ANGLE);
////                    Timer.delay(0.1);
////                    m_intake.setCargoIntakeOutput(Constants.CARGO_HOLD_SPEED);
//                } else {
//                    m_intake.setCargoIntakeOutput(0);
//                }
                break;
            case 1:
                m_intake.setHatchIntakeOutput(Constants.HATCH_GROUND_HOLD_SPEED);
                break;
            case 0:
            default:
                //m_intake.setHarpoonSecure(true);
                Timer.delay(0.25);
                m_intake.setHatchIntakeOutput(0);
                break;
        }

        Intake.overridePassive = false;
    }
}
