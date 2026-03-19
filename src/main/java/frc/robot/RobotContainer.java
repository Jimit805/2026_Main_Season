package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.commands.Intake.OscillateIntake;
import frc.robot.commands.Intake.ToggleIntake;
import frc.robot.commands.Shooter.AlignAndShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

@SuppressWarnings("unused")
public class RobotContainer {

    private final XboxController driverController = Buttons.controller1;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Drivetrain requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SendableChooser<Command> autoChooser;

    // ==========================
    // Subsystems
    // ==========================

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    public final Intake m_Intake;
    public final Indexer m_Indexer;
    public final Shooter m_Shooter;

    // ==========================
    // Commands
    // ==========================

    public final ToggleIntake m_ToggleIntake;
    public final OscillateIntake m_OscillateIntake;

    public final SpinStageOne m_spinStageOne;
    public final SpinStageTwo m_spinStageTwo;

    public final AlignAndShoot m_alignAndShootHub;
    public final AlignAndShoot m_alignAndPassLeft;
    public final AlignAndShoot m_alignAndPassRight;

    public RobotContainer() {
        m_Intake = new Intake();
        m_Indexer = new Indexer();
        m_Shooter = new Shooter(m_drivetrain);

        m_spinStageOne = new SpinStageOne(m_Indexer, Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
        m_spinStageTwo = new SpinStageTwo(m_Indexer, Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED);

        m_ToggleIntake = new ToggleIntake(m_Intake, m_Indexer);
        m_OscillateIntake = new OscillateIntake(m_Intake);

        m_alignAndShootHub = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.HUB, driverController);
        m_alignAndPassLeft = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_LEFT, driverController);
        m_alignAndPassRight = new AlignAndShoot(m_Shooter, m_Indexer, m_drivetrain, AlignAndShoot.Target.PASS_RIGHT, driverController);

        NamedCommands.registerCommand("DeployIntake", m_ToggleIntake);
        NamedCommands.registerCommand("Shoot", new ParallelCommandGroup(m_alignAndShootHub, m_OscillateIntake));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Default drive command — left stick translates, right stick X rotates
        m_drivetrain.setDefaultCommand(
                m_drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        // Hold neutral mode while disabled
        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true));

        // Reset field-centric heading
        Buttons.controller1_minusButton.onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        // SysId (back + start combos)
        // Buttons.controller1_AButton.whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // Buttons.controller1_BButton.whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // Buttons.controller1_YButton.whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // Buttons.controller1_XButton.whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Telemetry
        m_drivetrain.registerTelemetry(logger::telemeterize);

        // Intake Toggle
        Buttons.controller1_LeftTrigger.toggleOnTrue(m_ToggleIntake);

        // Hub Shooting
        Buttons.controller1_RightTrigger.whileTrue(new ParallelCommandGroup(m_alignAndShootHub, m_OscillateIntake));

        // Passing
        Buttons.controller1_rightBumper.whileTrue(new ParallelCommandGroup(m_alignAndPassLeft, m_OscillateIntake));
        Buttons.controller1_leftBumper.whileTrue(new ParallelCommandGroup(m_alignAndPassRight, m_OscillateIntake));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
