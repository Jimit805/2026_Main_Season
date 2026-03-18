// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AlignAndShoot extends Command {

    public enum Target {
        HUB, PASS_LEFT, PASS_RIGHT
    }

    private final Shooter m_shooter;
    private final Indexer m_indexer;
    private final Drivetrain m_drivetrain;
    private final Target m_target;

    private Translation2d targetPosition;

    public AlignAndShoot(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Target target) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_drivetrain = drivetrain;
        m_target = target;
        addRequirements(m_shooter, m_indexer);
    }

    @Override
    public void initialize() {
        // Pick target based on alliance
        boolean isBlue = DriverStation.getAlliance()
                .orElse(Alliance.Blue) == Alliance.Blue;

        targetPosition = switch (m_target) {
            case HUB -> isBlue
                    ? Constants.AlignTargets.BLUE_HUB
                    : Constants.AlignTargets.RED_HUB;
            case PASS_LEFT -> isBlue
                    ? Constants.AlignTargets.BLUE_PASS_LEFT
                    : Constants.AlignTargets.RED_PASS_LEFT;
            case PASS_RIGHT -> isBlue
                    ? Constants.AlignTargets.BLUE_PASS_RIGHT
                    : Constants.AlignTargets.RED_PASS_RIGHT;
        };

        // Pre-calculate shot params
        double dist = m_drivetrain.getDistanceTo(targetPosition);
        m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist));

    }

    @Override
    public void execute() {
        // Recompute every loop — robot is moving so compensation changes continuously
        Translation2d compensated = m_drivetrain.getCompensatedTarget(targetPosition);
        double dist = m_drivetrain.getDistanceTo(compensated);

        // Recalculate shot params against the compensated target each loop
        m_shooter.calculateShot(dist, m_shooter.getHorizontalVelocity(dist));

        // Aim at compensated point, not real hub
        m_drivetrain.setRotationOverride(() -> m_drivetrain.getHeadingPIDOutput(compensated));

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
        m_drivetrain.clearRotationOverride(); // Give rotation back to the driver
        m_shooter.spinFlywheels(0);
        m_shooter.pivotShooter(Constants.ShooterConstants.SHOOTER_STOW);
        m_indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}