package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.slicelibs.math.Conversions;

public class SimSwerveModuleIO implements SwerveModuleIO {

    private final DCMotorSim driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, Constants.DriveConstants.DRIVE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));
    private final DCMotorSim angleMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 0.004, Constants.DriveConstants.ANGLE_GEAR_RATIO),
            DCMotor.getKrakenX44(1));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0,
            Constants.DriveConstants.DRIVE_KV);
    private final PIDController drivePID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController anglePID = new PIDController(0.5, 0.0, 0.0);

    private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
    private double driveAppliedVolts = 0.0;
    private double angleAppliedVolts = 0.0;

    public SimSwerveModuleIO() {
        anglePID.enableContinuousInput(-180, 180);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveMotor.update(0.02);
        angleMotor.update(0.02);

        inputs.drivePositionMeters = driveMotor.getAngularPositionRotations()
                * Constants.DriveConstants.WHEEL_CIRCUMFERENCE;
        inputs.driveVelocityMetersPerSec = Conversions.RPMToMPS(driveMotor.getAngularVelocityRPM(),
                Constants.DriveConstants.WHEEL_CIRCUMFERENCE);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveMotor.getCurrentDrawAmps());

        inputs.absoluteAnglePosition = new Rotation2d(angleMotor.getAngularPositionRad())
                .plus(turnAbsoluteInitPosition);
        inputs.integratedAnglePosition = new Rotation2d(angleMotor.getAngularPositionRad());
        inputs.angleVelocityDegreesPerSec = edu.wpi.first.math.util.Units
                .radiansToDegrees(angleMotor.getAngularVelocityRadPerSec());
        inputs.angleAppliedVolts = angleAppliedVolts;
        inputs.angleCurrentAmps = Math.abs(angleMotor.getCurrentDrawAmps());
    }

    @Override
    public void setDriveVelocity(double velocity) {
        driveAppliedVolts = MathUtil.clamp(drivePID.calculate(Conversions.RPMToMPS(driveMotor.getAngularVelocityRPM(),
                Constants.DriveConstants.WHEEL_CIRCUMFERENCE), velocity) + driveFeedforward.calculate(velocity), -12,
                12);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setAnglePosition(double degrees) {
        angleAppliedVolts = MathUtil.clamp(
                anglePID.calculate(angleMotor.getAngularPositionRotations() * 360, degrees), -12.0, 12.0);
        angleMotor.setInputVoltage(angleAppliedVolts);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setAngleVoltage(double volts) {
        angleAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        angleMotor.setInputVoltage(angleAppliedVolts);
    }
}