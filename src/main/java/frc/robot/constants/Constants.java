/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // USB PORTS
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;

    // CAN ADDRESSES
    public static final int pcmOne = 11;

    public static final int leftFrontDriveMotor = 20;
    public static final int leftRearDriveMotor = 21;
    public static final int rightFrontDriveMotor = 22;
    public static final int rightRearDriveMotor = 23;
    public static final int intakeMotor = 30;
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;
    public static final int colorWheelMotor = 45;
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;
    public static final int skyhookMotor = 55;
    public static final int turretMotor = 60;
    public static final int turretEncoder = 61;

    // PWM
    public static final int ledPort = 0;

    // DIO
    public static final int intakeSensor = 0;
    public static final int indexerTopSensor = 1;
    public static final int indexerBottomSensor = 2;
    public static final int turretHomeSensor = 3;


    //Solenoid addresses
    public static final int driveTrainShiftersForward = 0;
    public static final int driveTrainShiftersReverse = 1;
    public static final int intakePistonForward = 2; // 2
    public static final int intakePistonReverse = 3; // 3
    public static final int climbPistonForward = 4;
    public static final int climbPistonReverse = 5;

    // Shoot on the Move
    public static final double g = 9.81; // Absolute value, in meters per second squared
    public static final double airResistanceCoefficient = 1.05; // Constant that shoot velocity is multiplied by to account for air resistance

    // Target measurements
    public static final double ballRadius = Units.inchesToMeters(3.5);
    public static final double ballTolerance = Units.inchesToMeters(1);
    public static final double outerTargetHeight = Units.inchesToMeters(30);
    public static final double targetOffset = Units.inchesToMeters(29.25);

    // Use actual values for these
    public static final double verticalTargetDistance = Units.inchesToMeters(98.25 - 38); // Distance between shooter and target heights from ground
    public static final double verticalShooterAngle = Math.PI / 3; // Angle ball is shot from shooter relative to the ground 
    public static final double tanSquaredVerticalShooterAngle = Math.pow(Math.tan(verticalShooterAngle), 2);
    public static final double sinOfVerticalShootAngle = Math.sin(verticalShooterAngle);
    public static final double cosOfVerticalShootAngle = Math.cos(verticalShooterAngle);
    public static final double targetXPosition = 0;
    public static final double targetYPosition = Units.inchesToMeters(629.25);

    public static final double turretAcceleration = .75; // radians per second
    public static final double shooterAcceleration = 1000; // RPM per second

    public static final double navXToShooterDistance = Units.inchesToMeters(1.4); // Meters
    public static final double navXToShooterAngle = 0; // Radians; angle offset between navX and shooter
    public static final double flywheelDiameter = 0.1; // Meters

    public static final double maxShooterRPM = 3500; // Highest RPM that shooter can launch balls at without breaking

    public static final class DriveConstants {
        public static final int[] kLeftEncoderPorts = new int[]{10, 11};
        public static final int[] kRightEncoderPorts = new int[]{12, 13};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        // Example values only -- use what's on your physical robot!
        public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(2);
        public static final double kDriveGearingLow = 7.49;
        public static final double kDriveGearingHigh = 14.14;


        public static final int kMagEncoderCPR = 4096;
        public static final int kFalconEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = Units.feetToMeters(0.5);
        public static final double kEncoderDistancePerPulseLow =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearingLow;
        public static final double kEncoderDistancePerPulseHigh =
                // Encoders are not on the wheel shaft for Falcons, so need to multiply by gear ratio
                (kWheelDiameterMeters * Math.PI) / (double) kFalconEncoderCPR * kDriveGearingLow;
        public static final double kEncoderDistancePerPulseSim =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kMagEncoderCPR * kDriveGearingHigh;

        public static final boolean kGyroReversed = true;

        public static final boolean inSlowGear = true; // True = 1 : 14.14, False = 1 : 7.49

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.675;//inSlowGear ? 0.683 : 0.81;
        public static final double kvVoltSecondsPerMeter = 3.21;//inSlowGear ? 3.19 : 1.74;
        public static final double kaVoltSecondsSquaredPerMeter = 0.256;//inSlowGear ? 0.227 : 0.301;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // These two values are "angular" kV and kA
        public static final double kvVoltSecondsPerRadian = 3.34;//inSlowGear ? 3.41 : 2.08; // originally 1.5
        public static final double kaVoltSecondsSquaredPerRadian = 0.19;//inSlowGear ? 0.111 : -0.0132; // originally 0.3

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
                LinearSystemId.identifyDrivetrainSystem(kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter,
                        kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);
    }
}
