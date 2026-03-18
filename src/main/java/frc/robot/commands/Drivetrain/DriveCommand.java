package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Buttons;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.configs.JoystickFilterConfig;

@SuppressWarnings("unused")
public class DriveCommand extends Command {

    private final Drivetrain m_drivetrain;
    private final XboxController m_driverController;
    private final PolarJoystickFilter translationFilter, rotationFilter;
    private final boolean m_isOpenLoop;
    private boolean m_isFieldRelative = true;

    public DriveCommand(Drivetrain drivetrain, XboxController driverController, boolean isOpenLoop) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        m_driverController = driverController;
        m_isOpenLoop = isOpenLoop;

        translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.04, 0.9, 1.0, 1.0));
        rotationFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.04, 0.6, 1.0, 1.0));
    }

    @Override
    public void initialize() {
        m_drivetrain.runDutyCycle(0, 0);
    }

    @Override
    public void execute() {
        // Left stick -> translation, right stick X -> rotation
        double[] translation = translationFilter.filter(
                -m_driverController.getRawAxis(0), // left stick X (strafe)
                -m_driverController.getRawAxis(1)); // left stick Y (forward)

        double translationX = translation[0] * Constants.DriveConstants.MAX_LINEAR_VELOCITY;
        double translationY = translation[1] * Constants.DriveConstants.MAX_LINEAR_VELOCITY;

        double rotation = rotationFilter.filter(-m_driverController.getRawAxis(4), 0)[0]
                * Constants.DriveConstants.MAX_ANGULAR_VELOCITY;

        // Hold left bumper for robot-relative drive at half speed
        /*
        m_isFieldRelative = !Buttons.controller1_leftBumper.getAsBoolean();
        if (!m_isFieldRelative) {
            translationX *= 0.5;
            translationY *= 0.5;
            rotation *= 0.5;
        }
        */

        m_drivetrain.drive(new Transform2d(new Translation2d(translationX, translationY), new Rotation2d(rotation)),
                m_isOpenLoop, m_isFieldRelative);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(new Transform2d(), m_isOpenLoop, m_isFieldRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}