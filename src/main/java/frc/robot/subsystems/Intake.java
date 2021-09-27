package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/*
Subsystem for interacting with the robot's intake
 */

public class Intake extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    // PID and FeedForward loop terms
    private final double kFF = 0.00068; //0.06; //0.122
    private final double kP = 6e-5; //0.492
    private final double kI = 0;
    private final double kD = 0;

    private final double kI_Zone = 0;
    private final double allowableError = 50;
    private final double maxVel = 5880;
    private final double maxAccel = 58800;
    private final double gearRatio = 1.0 / 3.0;
    // Intake motor setup
    // private final TalonFX intakeMotor = new TalonFX(Constants.intakeMotor);
    DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);
    //  private CANEncoder intakeEncoder = intakeMotor.getEncoder();
//  private CANPIDController canPidController = intakeMotor.getPIDController();
    private boolean intaking = false;

    public Intake() {
        // intakeMotor.configFactoryDefault();
        // intakeMotor.setNeutralMode(NeutralMode.Brake);
        // intakeMotor.setInverted(false);

//    canPidController.setFF(kFF);
//    canPidController.setP(kP);
//    canPidController.setI(kI);
//    canPidController.setIZone(kI_Zone);
//    canPidController.setD(kD);
//    canPidController.setSmartMotionMaxVelocity(maxVel, 0);
//    canPidController.setSmartMotionMaxAccel(maxAccel, 0);
//    canPidController.setSmartMotionAllowedClosedLoopError(allowableError, 0);
    }

  // Self-explanatory functions
  
  public boolean getIntakingState() {
    return intaking;
  }

    public void setIntakingState(boolean state) {
        intaking = state;
    }

    public boolean getIntakePistonExtendStatus() {
        return intakePiston.get() == DoubleSolenoid.Value.kForward;
    }

    public void setintakePiston(boolean state) {
        intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void setIntakePercentOutput(double value) {
        // intakeMotor.set(ControlMode.PercentOutput, value);
    }

//  public double getRPM(){
//    return intakeEncoder.getVelocity() * gearRatio;
//  }
//
//  public void setDirectRPM(double rpm){
//    canPidController.setReference(rpm, ControlType.kSmartVelocity);
//  }
//
//  public void setRPM(double rpm){
//    double setpoint =  rpm / gearRatio;
//    canPidController.setReference(setpoint, ControlType.kSmartVelocity);
//  }

    private void updateSmartDashboard() {
        SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
        SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
    }

    @Override
    public void periodic() {
        updateSmartDashboard();
    }
}
