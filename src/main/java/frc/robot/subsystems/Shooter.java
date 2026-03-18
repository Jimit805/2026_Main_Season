// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.FullShooterParams;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.slicelibs.TalonFXPositionalSubsystem;

public class Shooter extends TalonFXPositionalSubsystem {

    private final Drivetrain m_drivetrain;
    private TalonFX leftShooterMotor, rightShooterMotor;
    private Follower rightFollowerRequest;
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    // Tuning fields
    private boolean tuningMode = false;
    private double tunedRPM = 3000.0;
    private double tunedHoodAngle = 20.0;

    private double targetSpeed, targetPosition;

    /** Creates a new Shooter. */ // Parameter drivetrain is required to find robot velocity for SWIM calculations
    public Shooter(Drivetrain drivetrain) {
        super(new int[] { Constants.ShooterConstants.PIVOT_MOTOR_ID },
                new boolean[] { true },
                Constants.ShooterConstants.AIM_KP,
                Constants.ShooterConstants.AIM_KI,
                Constants.ShooterConstants.AIM_KD,
                Constants.ShooterConstants.AIM_KG,
                Constants.ShooterConstants.PIVOT_GEAR_RATIO,
                GravityTypeValue.Arm_Cosine,
                Constants.ShooterConstants.POSITION_CONVERSION_FACTOR,
                Constants.ShooterConstants.VELOCITY_CONVERSION_FACTOR,
                Constants.CTRE_CONFIGS.pivotConfigs);
        setEncoderPosition(Constants.ShooterConstants.SHOOTER_STOW); // 12 degree from ground stow angle
        
        // Tuning mode defaults
        SmartDashboard.putBoolean("Shooter/TuningMode", false);
        SmartDashboard.putNumber("Shooter/TunedRPM", 3000.0);
        SmartDashboard.putNumber("Shooter/TunedHoodAngle", 20.0);

        // Define the motors for spinning the flywheels.
        leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
        rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);

        // Set the motor configs.
        leftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
        rightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
        rightFollowerRequest = new Follower(leftShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        m_drivetrain = drivetrain;

    }

    // Set flywheels to a specific speed
    public void spinFlywheels(double targetRPM) {
        leftShooterMotor.setControl(flywheelVelocityRequest.withVelocity(targetRPM / 60.0));
        rightShooterMotor.setControl(rightFollowerRequest);
    }

    // Move shooter hood to a position
    public void pivotShooter(double angle) {
        setPosition(angle);
    }

    public double getHorizontalVelocity(double distance, Translation2d target) {
        FullShooterParams params = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
        double baselineHorizVel = distance / params.tof();

        // Project robot's field-relative velocity onto the robot to target vector
        ChassisSpeeds fieldSpeeds = m_drivetrain.getFieldRelativeSpeeds();
        Translation2d toTarget = target.minus(m_drivetrain.getPose().getTranslation()).getNorm() > 0
                ? target.minus(m_drivetrain.getPose().getTranslation())
                : new Translation2d(1, 0);
        Translation2d unitVec = toTarget.div(toTarget.getNorm());
        double robotVelAlongTarget = fieldSpeeds.vxMetersPerSecond * unitVec.getX()
                + fieldSpeeds.vyMetersPerSecond * unitVec.getY();

        // Moving toward target means ball needs less velocity
        return baselineHorizVel - robotVelAlongTarget;
    }

    public void calculateShot(double distance, double requiredVelocity) {
        FullShooterParams baseline = Constants.ShooterConstants.SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.tof();

        // Clamp ratio so we never get NaN or nonsensical values
        double velocityRatio = MathUtil.clamp(requiredVelocity / baselineVelocity, 0.5, 2.0);

        // Scale RPM proportionally — more velocity needed means more RPM
        targetSpeed = baseline.rpm() * velocityRatio;

        // Hood angle: derive from the total velocity vector totalVelocity =
        // baselineHoriz / cos(hoodAngle)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle()));
        double targetHoriz = MathUtil.clamp(requiredVelocity, 0.0, totalVelocity);
        targetPosition = Math.toDegrees(Math.acos(targetHoriz / totalVelocity));
    }

    public double getFlywheelSpeed() {
        return (leftShooterMotor.getVelocity().getValueAsDouble() + rightShooterMotor.getVelocity().getValueAsDouble())
                / 2 * 60.0;
    }

    public double getPivotPosition() {
        return getPositions()[0];
    }

    public boolean atTargetSpeed() {
        return Math.abs(targetSpeed - getFlywheelSpeed()) <= Constants.ShooterConstants.FLYWHEEL_RPM_ACCEPTABLE_ERROR;
    }

    public boolean atTargetPosition() {
        return Math.abs(targetPosition
                - getPivotPosition()) <= (Constants.ShooterConstants.VERTICAL_AIM_ACCEPTABLE_ERROR * (Math.PI / 180));
    }

    public boolean isTuningMode() {
        return tuningMode;
    }

    // Determines if the hub is active
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    // Determines if the hub is 4 seconds from active
    public boolean isHubAlmostActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > (130 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > (105 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > (80 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > (55 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > (30 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Always publish actual state
        SmartDashboard.putNumber("Shooter/ActualRPM", getFlywheelSpeed());
        SmartDashboard.putNumber("Shooter/TargetRPM", targetSpeed);
        SmartDashboard.putNumber("Shooter/TargetHoodAngle", targetPosition);
        SmartDashboard.putNumber("Shooter/ActualHoodAngle", getPivotPosition());
        SmartDashboard.putBoolean("Shooter/AtSpeed", atTargetSpeed());
        SmartDashboard.putBoolean("Shooter/AtPosition", atTargetPosition());

        // Tuning mode — read overrides from dashboard
        tuningMode = SmartDashboard.getBoolean("Shooter/TuningMode", false);
        SmartDashboard.putBoolean("Shooter/TuningMode", tuningMode);

        if (tuningMode) {
            tunedRPM = SmartDashboard.getNumber("Shooter/TunedRPM", tunedRPM);
            tunedHoodAngle = SmartDashboard.getNumber("Shooter/TunedHoodAngle", tunedHoodAngle);

            // Push defaults back so the fields appear on first run
            SmartDashboard.putNumber("Shooter/TunedRPM", tunedRPM);
            SmartDashboard.putNumber("Shooter/TunedHoodAngle", tunedHoodAngle);

            // Override target params
            targetSpeed = tunedRPM;
            targetPosition = tunedHoodAngle;
        }
    }
}
