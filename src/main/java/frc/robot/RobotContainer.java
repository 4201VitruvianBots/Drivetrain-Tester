/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autonomous.routines.AutoNavBarrel;
import frc.robot.commands.drivetrain.TalonFXSetArcadeDrive;
import frc.robot.constants.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.vitruvianlib.utils.JoystickWrapper;
import frc.vitruvianlib.utils.XBoxTrigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final PowerDistributionPanel pdp = new PowerDistributionPanel();
    private final DriveTrain m_driveTrain = new DriveTrain(pdp);

    static final JoystickWrapper leftJoystick = new JoystickWrapper(Constants.leftJoystick);
    static final JoystickWrapper rightJoystick = new JoystickWrapper(Constants.rightJoystick);
    static final JoystickWrapper xBoxController = new JoystickWrapper(Constants.xBoxController);
    public final Button[] leftButtons = new Button[2];
    public final Button[] rightButtons = new Button[2];
    public final Button[] xBoxButtons = new Button[10];
    public final Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    static JoystickWrapper testController = new JoystickWrapper(4);
    public Button[] testButtons = new Button[10];

    private static boolean init = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();
        // Configure the button bindings
        configureButtonBindings();
    }

    public static boolean getInitializationState() {
        return init;
    }

    public static void setInitializationState(boolean state) {
        init = state;
    }

    public void initializeSubsystems() {
        m_driveTrain.setDefaultCommand(new TalonFXSetArcadeDrive(m_driveTrain, () -> leftJoystick.getRawAxis(1),
                () -> -rightJoystick.getRawAxis(0)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        leftJoystick.invertRawAxis(1, true);
        rightJoystick.invertRawAxis(0, true);
        xBoxController.invertRawAxis(1, true);
        xBoxController.invertRawAxis(5, true);
        for (int i = 0; i < leftButtons.length; i++)
            leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
        for (int i = 0; i < rightButtons.length; i++)
            rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
        for (int i = 0; i < xBoxButtons.length; i++)
            xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
        for (int i = 0; i < xBoxPOVButtons.length; i++)
            xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));
        xBoxLeftTrigger = new XBoxTrigger(xBoxController, 2);
        xBoxRightTrigger = new XBoxTrigger(xBoxController, 3);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     */

    public void disabledInit() {
        setInitializationState(true);
        m_driveTrain.setDriveTrainNeutralMode(2);
    }

    public void robotPeriodic() {

    }

    public void teleOpInit() {
        if (RobotBase.isReal()) {
            m_driveTrain.resetEncoderCounts();
            m_driveTrain.setDriveTrainNeutralMode(2); // All in coast; change this maybe
        } else {
            m_driveTrain.resetEncoderCounts();
        }
    }

    public void teleOpPeriodic() {

    }

    public Command getAutonomousCommand() {
        return new AutoNavBarrel(m_driveTrain);
    }

    public void autonomousPeriodic() {
    }

    public DriveTrain getRobotDrive() {
        return m_driveTrain;
    }
}
