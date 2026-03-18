package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.Drivetrain.DriveCommand;
import frc.robot.commands.Drivetrain.ResetFieldOrientedHeading;
import frc.robot.commands.Drivetrain.RunDutyCycleCommand;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.commands.Intake.OscillateIntake;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Shooter.AlignAndShoot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.RealSwerveModuleIO;
import frc.robot.subsystems.Drivetrain.SimSwerveModuleIO;
import frc.robot.subsystems.Drivetrain.SwerveModuleIO;

public class RobotContainer {

    private final XboxController driverController = Buttons.controller1;

    // ==========================
    // Subsystems
    // ==========================

    public final Drivetrain m_drivetrain;
    public final Intake m_Intake;
    public final Indexer m_Indexer;
    public final Shooter m_Shooter;

    // ==========================
    // Commands
    // ==========================

    /* Drivetrain */
    public final DriveCommand m_swerveDriveOpenLoop;
    public final DriveCommand m_swerveDriveClosedLoop;
    public final RunDutyCycleCommand m_driveDutyCycle;
    public final ResetFieldOrientedHeading m_resetFieldOrientedHeading;
    public final Command m_sysIDDriveRoutine;

    /* Intake */
    public final ToggleIntake m_ToggleIntake;
    public final OscillateIntake m_OscillateIntake;

    /* Indexer */
    public final SpinStageOne m_spinStageOne;
    public final SpinStageTwo m_spinStageTwo;

    /* Shooter */
    public final AlignAndShoot m_alignAndShootHub;
    public final AlignAndShoot m_alignAndPassLeft;
    public final AlignAndShoot m_alignAndPassRight;

    public RobotContainer() {

        // ==========================
        // Subsystems
        // ==========================

        switch (Constants.ADVANTAGE_KIT_MODE) {
            case REAL:
                m_drivetrain = new Drivetrain(
                        new RealSwerveModuleIO(Constants.DriveConstants.FRONT_LEFT_MODULE),
                        new RealSwerveModuleIO(Constants.DriveConstants.FRONT_RIGHT_MODULE),
                        new RealSwerveModuleIO(Constants.DriveConstants.BACK_RIGHT_MODULE),
                        new RealSwerveModuleIO(Constants.DriveConstants.BACK_LEFT_MODULE));
                break;
            case SIM:
                m_drivetrain = new Drivetrain(
                        new SimSwerveModuleIO(),
                        new SimSwerveModuleIO(),
                        new SimSwerveModuleIO(),
                        new SimSwerveModuleIO());
                break;
            default:
                m_drivetrain = new Drivetrain(
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        });
                break;
        }

        m_Intake = new Intake();
        m_Indexer = new Indexer();
        m_Shooter = new Shooter();

        // ==========================
        // Commands
        // ==========================

        /* Drivetrain */
        m_swerveDriveOpenLoop = new DriveCommand(m_drivetrain, driverController, true);
        m_swerveDriveClosedLoop = new DriveCommand(m_drivetrain, driverController, false);
        m_driveDutyCycle = new RunDutyCycleCommand(m_drivetrain, 0.10, 0);
        m_resetFieldOrientedHeading = new ResetFieldOrientedHeading(m_drivetrain);
        m_sysIDDriveRoutine = new DeferredCommand(m_drivetrain::getSysIDDriveRoutine, Set.of(m_drivetrain));

        /* Intake */
        m_ToggleIntake = new ToggleIntake(m_Intake, m_Indexer);
        m_OscillateIntake = new OscillateIntake(m_Intake);

        /* Indexer */
        m_spinStageOne = new SpinStageOne(m_Indexer, 0.2);
        m_spinStageTwo = new SpinStageTwo(m_Indexer, 0.2);

        /* Shooter */
        m_alignAndShootHub = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.HUB);
        m_alignAndPassLeft = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_LEFT);
        m_alignAndPassRight = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_RIGHT);

        configureBindings();

        m_drivetrain.setDefaultCommand(m_swerveDriveClosedLoop);
    }

    private void configureBindings() {

        // ================
        // Driver Controls
        // ================

        /* Drivetrain */
        // Buttons.controller1_YButton.onTrue(m_sysIDDriveRoutine);
        Buttons.controller1_minusButton.onTrue(m_resetFieldOrientedHeading);

        /* Shooter */
        Buttons.controller1_RightTrigger.whileTrue(m_alignAndShootHub);
        Buttons.controller1_rightBumper.whileTrue(m_alignAndPassLeft);
        Buttons.controller1_leftBumper.whileTrue(m_alignAndPassRight);

        /* Intake */
        Buttons.controller1_LeftTrigger.onTrue(m_ToggleIntake);
        Buttons.controller1_RightTrigger.whileTrue(m_OscillateIntake);

        // ==================
        // Operator Controls
        // ==================

    }

    public Command getAutonomousCommand() {
        return null; // TODO: add autos
    }
}