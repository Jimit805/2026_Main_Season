// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.slicelibs.PolarJoystickFilter;
import frc.slicelibs.configs.JoystickFilterConfig;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AlignAndShoot extends Command {

    public enum Target {
        HUB, PASS_LEFT, PASS_RIGHT
    }

    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Target m_target;
    private final Drivetrain m_drivetrain;
    private final XboxController m_driverController;
    private final PolarJoystickFilter translationFilter = new PolarJoystickFilter(
            new JoystickFilterConfig(0.04, 0.9, 1.0, 1.0));

    private Translation2d targetPosition;

    public AlignAndShoot(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Target target,
            XboxController driverController) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_target = target;
        m_drivetrain = drivetrain;
        // Needed to allow the driver override the drivetrain translation while shooting
        m_driverController = driverController;

        addRequirements(m_shooter, m_indexer, m_drivetrain);
    }

    @Override
    public void initialize() {
        // Pick target based on alliance
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

        targetPosition = switch (m_target) {
            case HUB -> isBlue ? Constants.AlignTargets.BLUE_HUB : Constants.AlignTargets.RED_HUB;
            case PASS_LEFT -> isBlue ? Constants.AlignTargets.BLUE_PASS_LEFT : Constants.AlignTargets.RED_PASS_LEFT;
            case PASS_RIGHT -> isBlue ? Constants.AlignTargets.BLUE_PASS_RIGHT : Constants.AlignTargets.RED_PASS_RIGHT;
        };

        // Pre-calculate shot params
        double dist = m_drivetrain.getDistanceTo(targetPosition);
        m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist, targetPosition));

    }

    @Override
    public void execute() {
        // Compensate target for drivetrain movement
        Translation2d compensated = m_drivetrain.getCompensatedTarget(targetPosition);
        double dist = m_drivetrain.getDistanceTo(compensated);

        // Recalculate shot params against the compensated target each loop
        if (!m_shooter.isTuningMode()) {
            m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist, compensated));
        }
        
        // Drive - Drive controls translation, PID controls rotation
        double[] translation = translationFilter.filter(
                -m_driverController.getRawAxis(0),
                -m_driverController.getRawAxis(1));
        m_drivetrain.drive(
                new Transform2d(
                        new Translation2d(translation[0] * Constants.DriveConstants.MAX_LINEAR_VELOCITY,
                                translation[1] * Constants.DriveConstants.MAX_LINEAR_VELOCITY),
                        new Rotation2d(m_drivetrain.getHeadingPIDOutput(compensated))), false, true);

        m_shooter.spinFlywheels(m_shooter.getTargetVelocity());

        m_shooter.spinFlywheels(m_shooter.getTargetVelocity());
        m_shooter.pivotShooter(m_shooter.getTargetPosition());

        if (m_drivetrain.atTargetHeading() && m_shooter.atTargetSpeed() && m_shooter.atTargetPosition()) {
            m_indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
            m_indexer.runStageTwoMotor(Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED);
        } else {
            m_indexer.stopAll();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.spinFlywheels(0);
        m_shooter.pivotShooter(Constants.ShooterConstants.SHOOTER_STOW);
        m_indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}