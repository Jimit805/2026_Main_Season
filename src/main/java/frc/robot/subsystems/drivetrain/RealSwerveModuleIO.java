package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.slicelibs.configs.SwerveModuleConstants;
import frc.slicelibs.math.Conversions;

public class RealSwerveModuleIO implements SwerveModuleIO {

    private final Rotation2d angleOffset;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedforward;

    /* Drive control requests */
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

    /* Turn control request - ContinuousWrap enabled in config */
    private final PositionVoltage turnPositionRequest = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut turnVoltageRequest = new VoltageOut(0).withEnableFOC(true);

    /* Drive status signals */
    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<AngularVelocity> driveVelocitySignal;
    private final StatusSignal<AngularAcceleration> driveAccelerationSignal;
    private final StatusSignal<Voltage> driveAppliedVoltsSignal;
    private final StatusSignal<Current> driveCurrentSignal;

    /* Turn status signals */
    private final StatusSignal<Angle> turnPositionSignal;
    private final StatusSignal<AngularVelocity> turnVelocitySignal;
    private final StatusSignal<Voltage> turnAppliedVoltsSignal;
    private final StatusSignal<Current> turnCurrentSignal;

    /* CANcoder signal */
    private final StatusSignal<Angle> cancoderPositionSignal;

    public RealSwerveModuleIO(SwerveModuleConstants moduleConstants) {
        this.angleOffset = moduleConstants.angleOffset;

        /* Drive motor */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        driveMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveDriveConfigs);
        driveMotor.getConfigurator().setPosition(0);
        driveFeedforward = new SimpleMotorFeedforward(
                Constants.DriveConstants.DRIVE_KS, Constants.DriveConstants.DRIVE_KV);

        /* CANcoder */
        angleEncoder = new CANcoder(moduleConstants.absoluteEncoderID);
        angleEncoder.getConfigurator().apply(Constants.CTRE_CONFIGS.swerveCANcoderConfig);
        cancoderPositionSignal = angleEncoder.getAbsolutePosition();

        /* Turn motor - seed from CANcoder, then apply config */
        turnMotor = new TalonFX(moduleConstants.angleMotorID);
        turnMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.m_swerveTurnConfigs);
        resetToAbsolute();

        /* Status signals */
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveAccelerationSignal = driveMotor.getAcceleration();
        driveAppliedVoltsSignal = driveMotor.getMotorVoltage();
        driveCurrentSignal = driveMotor.getSupplyCurrent();

        turnPositionSignal = turnMotor.getPosition();
        turnVelocitySignal = turnMotor.getVelocity();
        turnAppliedVoltsSignal = turnMotor.getMotorVoltage();
        turnCurrentSignal = turnMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(100, 
                drivePositionSignal, driveVelocitySignal,
                turnPositionSignal, turnVelocitySignal);
        BaseStatusSignal.setUpdateFrequencyForAll(50,
                driveAccelerationSignal, driveAppliedVoltsSignal, driveCurrentSignal,
                turnAppliedVoltsSignal, turnCurrentSignal);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePositionSignal, driveVelocitySignal,
                driveAccelerationSignal, driveAppliedVoltsSignal, driveCurrentSignal,
                turnPositionSignal, turnVelocitySignal,
                turnAppliedVoltsSignal, turnCurrentSignal);
        cancoderPositionSignal.refresh();

        inputs.drivePositionMeters = Conversions.talonToMeters(drivePositionSignal.getValueAsDouble(),
                Constants.DriveConstants.WHEEL_CIRCUMFERENCE, Constants.DriveConstants.DRIVE_GEAR_RATIO);
        inputs.driveVelocityMetersPerSec = Conversions.talonToMPS(driveVelocitySignal.getValueAsDouble(),
                Constants.DriveConstants.WHEEL_CIRCUMFERENCE, Constants.DriveConstants.DRIVE_GEAR_RATIO);
        inputs.driveAccelerationMetersPerSecSquared = Conversions.talonToMPSSquared(
                driveAccelerationSignal.getValueAsDouble(),
                Constants.DriveConstants.WHEEL_CIRCUMFERENCE, Constants.DriveConstants.DRIVE_GEAR_RATIO);
        inputs.driveAppliedVolts = driveAppliedVoltsSignal.getValueAsDouble();
        inputs.driveCurrentAmps = driveCurrentSignal.getValueAsDouble();

        inputs.absoluteAnglePosition = Rotation2d.fromRotations(cancoderPositionSignal.getValueAsDouble());
        // SensorToMechanismRatio is set in config, so motor position is already in
        // mechanism rotations
        inputs.integratedAnglePosition = Rotation2d.fromRotations(turnPositionSignal.getValueAsDouble());
        inputs.angleVelocityDegreesPerSec = turnVelocitySignal.getValueAsDouble() * 360.0;
        inputs.angleAppliedVolts = turnAppliedVoltsSignal.getValueAsDouble();
        inputs.angleCurrentAmps = turnCurrentSignal.getValueAsDouble();
    }

    @Override
    public void runDriveDutyCycle(double percentOutput) {
        driveDutyCycleRequest.Output = percentOutput;
        driveMotor.setControl(driveDutyCycleRequest);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveVoltageRequest.Output = volts;
        driveMotor.setControl(driveVoltageRequest);
    }

    @Override
    public void setDriveVelocity(double velocityMPS) {
        double motorRPS = Conversions.MPSToTalon(velocityMPS, Constants.DriveConstants.WHEEL_CIRCUMFERENCE,
                Constants.DriveConstants.DRIVE_GEAR_RATIO);
        driveVelocityRequest.Velocity = motorRPS;
        driveVelocityRequest.FeedForward = driveFeedforward.calculate(velocityMPS);
        driveMotor.setControl(driveVelocityRequest);
    }

    @Override
    public void runAngleDutyCycle(double percentOutput) {
        turnMotor.set(percentOutput);
    }

    @Override
    public void setAngleVoltage(double volts) {
        turnVoltageRequest.Output = volts;
        turnMotor.setControl(turnVoltageRequest);
    }

    @Override
    public void setAnglePosition(double degrees) {
        // SensorToMechanismRatio applied, so position is in mechanism rotations
        turnPositionRequest.Position = degrees / 360.0;
        turnMotor.setControl(turnPositionRequest);
    }

    @Override
    public void resetToAbsolute() {
        cancoderPositionSignal.refresh();
        // Set turn motor position in mechanism rotations (offset subtracted)
        double absoluteRotations = cancoderPositionSignal.getValueAsDouble() - angleOffset.getRotations();
        // Multiply by gear ratio because SensorToMechanismRatio divides it back out
        turnMotor.getConfigurator().setPosition(absoluteRotations * Constants.DriveConstants.ANGLE_GEAR_RATIO);
    }
}