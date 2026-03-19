package frc.slicelibs.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

/**
 * Central CTRE motor configurations for all non-drivetrain subsystems.
 * Drivetrain motor configs are now owned by TunerConstants / CTRE swerve.
 */
public class CTREConfigs {

    public final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration extenderConfigs = new TalonFXConfiguration();
    public final TalonFXConfiguration rollerConfigs = new TalonFXConfiguration();

    public final TalonFXConfiguration indexerConfigs = new TalonFXConfiguration();

    public CTREConfigs() {
        configureShooter();
        configureIntake();
        configureIndexer();
    }

    private void configureShooter() {
        shooterConfigs.Slot0.kP = Constants.ShooterConstants.FLYWHEEL_KP;
        shooterConfigs.Slot0.kI = Constants.ShooterConstants.FLYWHEEL_KI;
        shooterConfigs.Slot0.kD = Constants.ShooterConstants.FLYWHEEL_KD;
        shooterConfigs.Slot0.kS = Constants.ShooterConstants.FLYWHEEL_KS;
        shooterConfigs.Slot0.kV = Constants.ShooterConstants.FLYWHEEL_KV;
        shooterConfigs.Feedback.SensorToMechanismRatio = Constants.ShooterConstants.FLYWHEEL_GEAR_RATIO;
        shooterConfigs.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
        shooterConfigs.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
        shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfigs.CurrentLimits.StatorCurrentLimit = Constants.ShooterConstants.PIVOT_STATOR_CURRENT_LIMIT;
        pivotConfigs.CurrentLimits.SupplyCurrentLimit = Constants.ShooterConstants.PIVOT_SUPPLY_CURRENT_LIMIT;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    private void configureIntake() {
        extenderConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.EXTENDER_STATOR_CURRENT_LIMIT;
        extenderConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.EXTENDER_SUPPLY_CURRENT_LIMIT;
        extenderConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfigs.Feedback.SensorToMechanismRatio = Constants.IntakeConstants.ROLLER_GEAR_RATIO;
        rollerConfigs.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        rollerConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
        rollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    private void configureIndexer() {
        indexerConfigs.CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
        indexerConfigs.CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;
        indexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }
}
