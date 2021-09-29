// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.BaseTalon;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.unmanaged.Unmanaged;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PowerDistributionPanel;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.SerialPort;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
// import edu.wpi.first.wpilibj.util.Units;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.constants.Constants;
// import frc.robot.constants.Constants.DriveConstants;

// public class TalonFXDriveTrain extends SubsystemBase {
//     private final double gearRatioHigh = 1.0 / 14.14;
//     private final double gearRatioLow = 1.0 / 7.49;
//     private final double wheelDiameter = 0.5;

//     private final double kS = DriveConstants.ksVolts;
//     private final double kV = DriveConstants.kvVoltSecondsPerMeter;
//     private final double kA = DriveConstants.kaVoltSecondsSquaredPerMeter;;

//     public double kP = 1.94;// Constants.DriveConstants.inSlowGear ? 1.89 : 2.74; //1.33
//     public double kI = 0;
//     public double kD = 0;
//     public int controlMode = 0;

//     DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
//     DifferentialDriveOdometry odometry;
//     DifferentialDrivePoseEstimator m_poseEstimator;
//     SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

//     PIDController leftPIDController = new PIDController(kP, kI, kD);
//     PIDController rightPIDController = new PIDController(kP, kI, kD);

//     PowerDistributionPanel m_pdp;

//     private final TalonFX[] driveMotors = { new TalonFX(Constants.leftFrontDriveMotor),
//             new TalonFX(Constants.leftRearDriveMotor), new TalonFX(Constants.rightFrontDriveMotor),
//             new TalonFX(Constants.rightRearDriveMotor) };
//     double m_leftOutput, m_rightOutput;

//     private final boolean[] brakeMode = { true, false, true, false };

//     // Set up shifters
//     DoubleSolenoid driveTrainShifters = new DoubleSolenoid(Constants.pcmOne, Constants.driveTrainShiftersForward,
//             Constants.driveTrainShiftersReverse);
//     private boolean m_driveShifterState;

//     private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

//     private final DoubleSolenoid solenoid = new DoubleSolenoid(0, 0, 1);

//     private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

//     // // The left-side drive encoder
//     // private final Encoder m_leftEncoder =
//     // new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0],
//     // Constants.DriveConstants.kLeftEncoderPorts[1],
//     // Constants.DriveConstants.kLeftEncoderReversed);
//     //
//     // // The right-side drive encoder
//     // private final Encoder m_rightEncoder =
//     // new Encoder(Constants.DriveConstants.kRightEncoderPorts[0],
//     // Constants.DriveConstants.kRightEncoderPorts[1],
//     // Constants.DriveConstants.kRightEncoderReversed);
//     // private EncoderSim m_leftEncoderSim;
//     // private EncoderSim m_rightEncoderSim;

//     // Temporary until CTRE supports FalconFX in WPILib Sim
//     private final TalonSRX[] simMotors = new TalonSRX[4];

//     public DifferentialDrivetrainSim m_drivetrainSimulator;
//     private ADXRS450_GyroSim m_gyroAngleSim;

//     private DoubleSupplier joystickCorrector = null; // A turning joystick to adjust trajectories during tele-op

//     public TalonFXDriveTrain(PowerDistributionPanel pdp) {
//         // Set up DriveTrain motors
//         configureCtreMotors(driveMotors);

//         m_pdp = pdp;
//         // initShuffleboardValues();
//         odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
//         // poseEstimator = new DifferentialDrivePoseEstimator(
//         // m_gyro.getRotation2d(),
//         // new Pose2d(),
//         // VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01),
//         // VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)),
//         // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

//         if (RobotBase.isSimulation()) { // If our robot is simulated
//             for (int i = 0; i < 4; i++)
//                 simMotors[i] = new TalonSRX(24 + i);
//             configureCtreMotors(simMotors);
//             simMotors[0].setSensorPhase(true);
//             simMotors[2].setSensorPhase(false);

//             // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulseSim);
//             // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulseSim);

//             m_drivetrainSimulator = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
//                     DriveConstants.kDriveGearbox, DriveConstants.kDriveGearingHigh, DriveConstants.kTrackwidthMeters,
//                     Constants.DriveConstants.kWheelDiameterMeters / 2.0, null);// VecBuilder.fill(0, 0, 0.0001, 0.1,
//                                                                                // 0.1, 0.005, 0.005));

//             // m_drivetrainSimulator = new DifferentialDrivetrainSim(
//             // DriveConstants.kDriveGearbox,
//             // DriveConstants.kDriveGearingHigh,
//             // 8.28, // Moment of inertia. Need to get it from the CAD model
//             // 70.44, // 140 lbs
//             // DriveConstants.kTrackwidthMeters,
//             // Constants.DriveConstants.kWheelDiameterMeters / 2.0,
//             // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

//             // m_leftEncoderSim = new EncoderSim(m_leftEncoder);
//             // m_rightEncoderSim = new EncoderSim(m_rightEncoder);
//             m_gyroAngleSim = new ADXRS450_GyroSim(m_gyro);
//         }
//         SmartDashboard.putData("DT Subsystem", this);
//     }

//     public void configureCtreMotors(BaseTalon... motors) {
//         for (int i = 0; i < motors.length; i++) {
//             motors[i].configFactoryDefault();
//             // motor.configVoltageCompSaturation(12);
//             // motor.enableVoltageCompensation(true);
//             // motor.configGetSupplyCurrentLimit(30);
//             // motor.configPeakCurrentLimit(40);
//             // motor.configPeakCurrentDuration(1000);
//             // motor.enableCurrentLimit(true);

//             motors[i].configOpenloopRamp(0.1);
//             motors[i].configClosedloopRamp(0.1);
//             motors[i].setNeutralMode(NeutralMode.Coast);
//             motors[i].configForwardSoftLimitEnable(false);
//             motors[i].configReverseSoftLimitEnable(false);

//             if (motors[i] instanceof TalonFX) {
//                 driveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
//                 driveMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
//             } else if (motors[i] instanceof TalonSRX) {
//                 simMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
//                 simMotors[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
//             }
//         }

//         motors[0].setInverted(true);
//         motors[1].setInverted(true);
//         motors[2].setInverted(false);
//         motors[3].setInverted(false);

//         motors[0].setSensorPhase(false);
//         motors[2].setSensorPhase(false);

//         motors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
//         motors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
//         motors[1].setNeutralMode(NeutralMode.Brake);
//         motors[3].setNeutralMode(NeutralMode.Brake);

//         motors[1].configOpenloopRamp(0);
//         motors[3].configOpenloopRamp(0);
//     }

//     // Self-explanatory functions

//     public int getEncoderCount(int sensorIndex) {
//         return (int) driveMotors[sensorIndex].getSelectedSensorPosition();
//     }

//     public double getAngle() {
//         if (RobotBase.isReal())
//             return navX.getAngle();
//         else
//             return m_gyro.getAngle();
//     }

//     public double getHeading() {
//         if (RobotBase.isReal())
//             return Math.IEEEremainder(-navX.getAngle(), 360);
//         else
//             return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
//     }

//     public void resetAngle() {
//         navX.zeroYaw();
//     }

//     public void setNavXOffset(double angle) { // Angle in degrees
//         navX.setAngleAdjustment(angle);
//     }

//     public double getWheelDistanceMeters(int sensorIndex) {
//         double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;
//         // double gearRatio = gearRatioHigh;

//         if (RobotBase.isReal())
//             return (driveMotors[sensorIndex].getSelectedSensorPosition() / 2048.0) * gearRatio * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//         else {
//             return (simMotors[sensorIndex].getSelectedSensorPosition() / 4096.0) * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             // if(sensorIndex == 0)
//             // return m_leftEncoder.getDistance();
//             // else if(sensorIndex == 2)
//             // return m_rightEncoder.getDistance();
//             // else
//             // return 0;
//         }
//     }

//     // ???
//     public double getMotorInputCurrent(int motorIndex) {
//         return driveMotors[motorIndex].getSupplyCurrent();
//     }

//     public void resetEncoderCounts() {
//         driveMotors[0].setSelectedSensorPosition(0);
//         driveMotors[2].setSelectedSensorPosition(0);
//         if (RobotBase.isSimulation()) {
//             // m_leftEncoder.reset();
//             // m_rightEncoder.reset();;
//             simMotors[0].getSimCollection().addQuadraturePosition(0);
//             simMotors[2].getSimCollection().addQuadraturePosition(0);
//         }
//     }

//     public void setAutosJoystick(DoubleSupplier supplier) {
//         joystickCorrector = supplier;
//     }

//     public void setMotorArcadeDrive(double throttle, double turn) {
//         double leftPWM = throttle + turn;
//         double rightPWM = throttle - turn;

//         // if(rightPWM > 1.0) {
//         // leftPWM -= rightPWM - 1.0;
//         // rightPWM = 1.0;
//         // } else if(rightPWM < -1.0) {
//         // leftPWM -= rightPWM + 1.0;
//         // rightPWM = -1.0;
//         // } else if(leftPWM > 1.0) {
//         // rightPWM -= leftPWM - 1.0;
//         // leftPWM = 1.0;
//         // } else if(leftPWM < -1.0) {
//         // rightPWM -= leftPWM + 1.0;
//         // leftPWM = -1.0;
//         // }

//         // Normalization
//         double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
//         if (magnitude > 1.0) {
//             leftPWM *= 1.0 / magnitude;
//             rightPWM *= 1.0 / magnitude;
//         }

//         setMotorPercentOutput(leftPWM, rightPWM);
//     }

//     public void setMotorTankDrive(double leftOutput, double rightOutput) {
//         setMotorPercentOutput(leftOutput, rightOutput);
//     }

//     public void setVoltageOutput(double leftVoltage, double rightVoltage) {
//         var batteryVoltage = RobotController.getBatteryVoltage();
//         if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage)) > batteryVoltage) {
//             leftVoltage *= batteryVoltage / 12.0;
//             rightVoltage *= batteryVoltage / 12.0;
//         }
//         SmartDashboardTab.putNumber("DriveTrain", "Left Voltage", leftVoltage);
//         SmartDashboardTab.putNumber("DriveTrain", "Right Voltage", rightVoltage);
//         if (joystickCorrector == null) {
//             setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);
//         } else {
//             setMotorPercentOutput(leftVoltage / batteryVoltage + joystickCorrector.getAsDouble(),
//                     rightVoltage / batteryVoltage - joystickCorrector.getAsDouble());
//         }
//     }

//     private void setMotorPercentOutput(double leftOutput, double rightOutput) {
//         m_leftOutput = leftOutput;
//         m_rightOutput = rightOutput;
//         driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
//         driveMotors[2].set(ControlMode.PercentOutput, rightOutput);

//         if (RobotBase.isSimulation()) {
//             simMotors[0].set(ControlMode.PercentOutput, leftOutput);
//             simMotors[2].set(ControlMode.PercentOutput, rightOutput);
//         }
//     }

//     // Make motors neutral
//     public void setDriveTrainNeutralMode(int mode) {
//         switch (mode) {
//             case 2:
//                 for (var motor : driveMotors)
//                     motor.setNeutralMode(NeutralMode.Coast);
//                 break;
//             case 1:
//                 for (var motor : driveMotors)
//                     motor.setNeutralMode(NeutralMode.Brake);
//                 break;
//             case 0:
//             default:
//                 driveMotors[0].setNeutralMode(NeutralMode.Brake);
//                 driveMotors[1].setNeutralMode(NeutralMode.Coast);
//                 driveMotors[2].setNeutralMode(NeutralMode.Brake);
//                 driveMotors[3].setNeutralMode(NeutralMode.Coast);
//                 brakeMode[0] = true;
//                 brakeMode[1] = false;
//                 brakeMode[2] = true;
//                 brakeMode[3] = false;
//                 break;
//         }
//     }

//     public boolean getDriveShifterStatus() {
//         return m_driveShifterState;
//     }

//     public void setDriveShifterStatus(boolean state) {
//         m_driveShifterState = state;
//         // double gearRatio = state ? Constants.DriveConstants.kDriveGearingHigh :
//         // Constants.DriveConstants.kDriveGearingLow ;
//         // double kEncoderDistancePerPulse = state ?
//         // Constants.DriveConstants.kEncoderDistancePerPulseHigh :
//         // Constants.DriveConstants.kEncoderDistancePerPulseLow;

//         // m_drivetrainSimulator.setCurrentGearing(gearRatio);
//         // m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
//         // m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

//         driveTrainShifters.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
//     }

//     public DifferentialDriveWheelSpeeds getSpeeds() {
//         double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;
//         // double gearRatio = gearRatioHigh;
//         double leftMetersPerSecond = 0, rightMetersPerSecond = 0;

//         if (RobotBase.isReal()) {
//             // getSelectedSensorVelocity() returns values in units per 100ms. Need to
//             // convert value to RPS
//             leftMetersPerSecond = (driveMotors[0].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             rightMetersPerSecond = (driveMotors[2].getSelectedSensorVelocity() * 10.0 / 2048.0) * gearRatio * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//         } else {
//             // This is apparently causing issues to the sim where the robot does not fully
//             // reach the endpoint. My assumption
//             // at the moment is that we need to re-characterize the drivetrain to get new
//             // values. I believe David noticed this
//             // issue on the actual robot when he had it. By removing this, the robot sim
//             // will follow the trajectory perfectly.
//             // leftMetersPerSecond = m_leftEncoderSim.getRate();
//             // rightMetersPerSecond = m_rightEncoderSim.getRate();
//             // leftMetersPerSecond = (simMotors[0].getSelectedSensorVelocity() * 10.0 /
//             // 4096.0) * Math.PI * Units.feetToMeters(wheelDiameter);
//             // rightMetersPerSecond = (simMotors[2].getSelectedSensorVelocity() * 10.0 /
//             // 4096.0) * Math.PI * Units.feetToMeters(wheelDiameter);
//         }

//         return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
//     }

//     public double getTravelDistance() {
//         double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;
//         // double gearRatio = gearRatioHigh;
//         double leftMeters, rightMeters;

//         if (RobotBase.isReal()) {
//             leftMeters = (driveMotors[0].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             rightMeters = (driveMotors[2].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             return (leftMeters + rightMeters) / 2.0;
//         } else {
//             // leftMeters = m_leftEncoder.getDistance();
//             // rightMeters = m_rightEncoder.getDistance();
//             leftMeters = (simMotors[0].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             rightMeters = (simMotors[2].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI
//                     * Units.feetToMeters(wheelDiameter);
//             return (leftMeters + rightMeters) / 2.0;
//         }
//     }

//     public SimpleMotorFeedforward getFeedforward() {
//         return feedforward;
//     }

//     public Pose2d getRobotPose() {
//         return odometry.getPoseMeters();
//     }

//     public DifferentialDriveKinematics getDriveTrainKinematics() {
//         return kinematics;
//     }

//     public PIDController getLeftPIDController() {
//         return leftPIDController;
//     }

//     public PIDController getRightPIDController() {
//         return rightPIDController;
//     }

//     public void resetOdometry(Pose2d pose, Rotation2d rotation) {
//         if (RobotBase.isSimulation()) {
//             resetEncoderCounts();
//             m_drivetrainSimulator.setPose(pose);
//         }
//         setNavXOffset(rotation.getDegrees());
//         odometry.resetPosition(pose, rotation);
//         resetEncoderCounts();
//     }

//     // private void initShuffleboardValues() {
//     // // Need to verify that this works again
//     // Shuffleboard.getTab("Drive Train").addNumber("Left Encoder", () ->
//     // getEncoderCount(0));
//     // Shuffleboard.getTab("Drive Train").addNumber("Right Encoder", () ->
//     // getEncoderCount(2));
//     // Shuffleboard.getTab("Drive Train").addNumber("xCoordinate",
//     // () -> Units.metersToFeet(getRobotPose().getTranslation().getX()));
//     // Shuffleboard.getTab("Drive Train").addNumber("yCoordinate",
//     // () -> Units.metersToFeet(getRobotPose().getTranslation().getY()));
//     // Shuffleboard.getTab("Drive Train").addNumber("Angle", () ->
//     // getRobotPose().getRotation().getDegrees());
//     // Shuffleboard.getTab("Drive Train").addNumber("leftSpeed",
//     // () -> Units.metersToFeet(getSpeeds().leftMetersPerSecond));
//     // Shuffleboard.getTab("Drive Train").addNumber("rightSpeed",
//     // () -> Units.metersToFeet(getSpeeds().rightMetersPerSecond));

//     // Shuffleboard.getTab("Turret").addNumber("Robot Angle", navX::getAngle);
//     // }

//     private void updateSmartDashboard() {
//         if (RobotBase.isReal()) {
//             SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
//             SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
//             SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
//                     Units.metersToFeet(getRobotPose().getTranslation().getX()));
//             SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
//                     Units.metersToFeet(getRobotPose().getTranslation().getY()));
//             SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPose().getRotation().getDegrees());
//             SmartDashboardTab.putNumber("DriveTrain", "leftSpeed", Units.metersToFeet(getSpeeds().leftMetersPerSecond));
//             SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
//                     Units.metersToFeet(getSpeeds().rightMetersPerSecond));

//             SmartDashboardTab.putNumber("Turret", "Robot Angle", getAngle());
//         } else {
//             SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
//             SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
//             SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
//                     Units.metersToFeet(getRobotPose().getTranslation().getX()));
//             SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
//                     Units.metersToFeet(getRobotPose().getTranslation().getY()));
//             SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPose().getRotation().getDegrees());
//             SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
//                     Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
//             SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
//                     Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));

//             SmartDashboardTab.putNumber("Turret", "Robot Angle", getAngle());
//         }
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         odometry.update(Rotation2d.fromDegrees(getHeading()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));

//         // m_poseEstimator.update(m_gyro.getRotation2d(),
//         // getSpeeds(),
//         // getWheelDistanceMeters(0),
//         // getWheelDistanceMeters(2));

//         // Also apply vision measurements. We use 0.3 seconds in the past as an example
//         // -- on
//         // a real robot, this must be calculated based either on latency or timestamps.
//         // m_poseEstimator.addVisionMeasurement(
//         // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
//         // m_poseEstimator.getEstimatedPosition()),
//         // Timer.getFPGATimestamp() - 0.3);

//         updateSmartDashboard();
//     }

//     public double getDrawnCurrentAmps() {
//         return m_drivetrainSimulator.getCurrentDrawAmps();
//     }

//     double maxVel;

//     @Override
//     public void simulationPeriodic() {
//         // odometry.update(Rotation2d.fromDegrees(getHeading()),
//         // m_leftEncoder.getDistance(),
//         // m_rightEncoder.getDistance());

//         // To update our simulation, we set motor voltage inputs, update the simulation,
//         // and write the simulated positions and velocities to our simulated encoder and
//         // gyro.
//         // We negate the right side so that positive voltages make the right side
//         // move forward.

//         m_drivetrainSimulator.setInputs(m_leftOutput * RobotController.getBatteryVoltage(),
//                 m_rightOutput * RobotController.getBatteryVoltage());
//         m_drivetrainSimulator.update(0.040);

//         // m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
//         // m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
//         // m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
//         // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
//         // m_gyroAngleSim.set(-m_drivetrainSimulator.getHeading().getDegrees());

//         // For CTRE devices, you must call this function periodically for simulation
//         Unmanaged.feedEnable(40);
//         simMotors[0].getSimCollection()
//                 .addQuadraturePosition(distanceMetersToTalonSrxUnits(m_drivetrainSimulator.getLeftPositionMeters()));
//         simMotors[0].getSimCollection().setQuadratureVelocity(
//                 velocityMetersToTalonSrxUnits(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
//         simMotors[2].getSimCollection()
//                 .addQuadraturePosition(distanceMetersToTalonSrxUnits(m_drivetrainSimulator.getRightPositionMeters()));
//         simMotors[2].getSimCollection().setQuadratureVelocity(
//                 velocityMetersToTalonSrxUnits(m_drivetrainSimulator.getRightVelocityMetersPerSecond()));
//         m_gyroAngleSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

//         SmartDashboard.putNumber("Robot Heading", getHeading());
//         SmartDashboard.putNumber("Robot Angle", getAngle());
//         SmartDashboard.putNumber("L Encoder Count", simMotors[0].getSelectedSensorPosition());
//         SmartDashboard.putNumber("R Encoder Count", simMotors[2].getSelectedSensorPosition());
//         SmartDashboard.putNumber("L Encoder Rate", simMotors[0].getSelectedSensorVelocity());
//         SmartDashboard.putNumber("R Encoder Rate", simMotors[2].getSelectedSensorVelocity());

//         SmartDashboard.putNumber("L Output", m_leftOutput);
//         SmartDashboard.putNumber("R Output", m_rightOutput);
//         SmartDashboard.putNumber("L Encoder Distance", getWheelDistanceMeters(0));
//         SmartDashboard.putNumber("R Encoder Distance", getWheelDistanceMeters(2));
//         // SmartDashboard.putNumber("L Encoder Count", m_leftEncoder.get());
//         // SmartDashboard.putNumber("R Encoder Count", m_rightEncoder.get());
//         // SmartDashboard.putNumber("L Encoder Rate", m_leftEncoder.getRate());
//         // SmartDashboard.putNumber("R Encoder Rate", m_rightEncoder.getRate());

//         SmartDashboard.putBoolean("High Gear", getDriveShifterStatus());
//         SmartDashboard.putBoolean("CTRE Feed Enabled", Unmanaged.getEnableState());

//         // if(m_leftEncoder.getRate() > maxVel) {
//         // SmartDashboard.putNumber("Max Vel", m_leftEncoder.get());
//         // maxVel = m_leftEncoder.getRate();
//         // }
//     }

//     int distanceMetersToTalonSrxUnits(double meters) {
//         // To simplify, for simulating Talons, pretend they are on the wheel axel (e.g.
//         // don't care about the gear ratio)
//         return (int) (meters * 4096.0 / (Units.feetToMeters(wheelDiameter) * Math.PI));
//     }

//     int velocityMetersToTalonSrxUnits(double meters) {
//         // To simplify, for simulating Talons, pretend they are on the wheel axel (e.g.
//         // don't care about the gear ratio)
//         return (int) (meters * 4096.0 / (Units.feetToMeters(wheelDiameter) * Math.PI * 10.0));
//     }

//     int distanceMetersToFalconFxUnits(double meters) {
//         double gearRatio = getDriveShifterStatus() ? gearRatioHigh : gearRatioLow;

//         return (int) (meters * 2048.0 / (10 * gearRatio * Math.PI));
//     }
// }
