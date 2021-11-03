package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

/*  The point of this class was to do two things:
    1) To simplify the construction of a Ramsete Command to follow a path.
    2) To ensure that the command actually ends once you are at your target

    The second point was due to not being sure how the RamseteCommand would behave since it was just introduced in 2020.
    May revisit the need to specify our own end conditions in the future.
 */
public class VitruvianRamseteCommand extends RamseteCommand {
    private final Trajectory m_trajectory;
    private final DriveTrain m_driveTrain;

    public VitruvianRamseteCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController controller, SimpleMotorFeedforward feedforward, DifferentialDriveKinematics kinematics, Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds, PIDController leftController, PIDController rightController, BiConsumer<Double, Double> outputVolts, DriveTrain driveTrain) {
        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts, driveTrain);
        m_driveTrain = driveTrain;
        m_trajectory = trajectory;

        int trajectorySize = m_trajectory.getStates().size() - 1;
        Pose2d m_finalPose = m_trajectory.getStates().get(trajectorySize).poseMeters;
    }

    @Override
    public void initialize() {
        super.initialize();
        double autoStartTime = Timer.getFPGATimestamp();
        double autoDuration = m_trajectory.getTotalTimeSeconds() + 1;
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboardTab.putNumber("DriveTrain", "Velocity", Units.metersToFeet(m_driveTrain.getDriveTrainKinematics().toChassisSpeeds(m_driveTrain.getSpeeds()).vxMetersPerSecond));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        SmartDashboardTab.putBoolean("DriveTrain", "isRunning", false);
    }
}
