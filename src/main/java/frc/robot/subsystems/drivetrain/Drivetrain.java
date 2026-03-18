package frc.robot.subsystems.Drivetrain;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Drivetrain extends SubsystemBase {

    private final SwerveModule[] swerveMods;

    // For sim heading delta tracking
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private final SwerveDrivePoseEstimator m_odometry;
    private final Pigeon2 m_gyro;
    private final StatusSignal<Angle> gyroYawSignal;
    private final StatusSignal<AngularVelocity> gyroYawVelocitySignal;
    public final Field2d m_field2d;
    private DoubleSupplier rotationOverride = null;

    private Rotation2d fieldOrientedOffset;
    private Rotation2d simHeading = new Rotation2d();

    private final SysIdRoutine sysIDDriveRoutine;
    public final SendableChooser<Command> sysIDChooser;

    public Drivetrain(SwerveModuleIO mod0IO, SwerveModuleIO mod1IO, SwerveModuleIO mod2IO, SwerveModuleIO mod3IO) {

        swerveMods = new SwerveModule[] {
                new SwerveModule(mod0IO, 0),
                new SwerveModule(mod1IO, 1),
                new SwerveModule(mod2IO, 2),
                new SwerveModule(mod3IO, 3)
        };

        m_gyro = new Pigeon2(Constants.DriveConstants.GYRO_ID);
        gyroYawSignal = m_gyro.getYaw();
        gyroYawVelocitySignal = m_gyro.getAngularVelocityZWorld();

        resetModulesToAbsolute();
        
        while (!m_gyro.isConnected()) {}
        m_gyro.reset();

        m_field2d = new Field2d();
        SmartDashboard.putData(m_field2d);

        DriverStation.waitForDsConnection(60);

        resetHeading(Rotation2d.fromDegrees(
                DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180));

        m_odometry = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kSwerveKinematics,
                getHeading(),
                getModulePositions(),
                new Pose2d(0, 0, Rotation2d.fromDegrees(
                        DriverStation.getAlliance().get() == Alliance.Blue ? 180 : 0)),
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.3, 0.3, 0.3));

        fieldOrientedOffset = Rotation2d.fromDegrees(
                DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180);

        PathPlannerLogging.setLogActivePathCallback(path -> {
            Logger.recordOutput("Odometry/Trajectory", path.toArray(new Pose2d[0]));
            addField2dTrajectory(path, "Trajectory");
        });
        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Odometry/Trajectory Setpoint", pose));

        sysIDDriveRoutine = new SysIdRoutine(
                new Config(),
                new Mechanism(
                        voltage -> {
                            for (SwerveModule mod : swerveMods) {
                                mod.runCharacterization(voltage.in(Units.Volts));
                            }
                        },
                        log -> {
                            for (SwerveModule mod : swerveMods) {
                                log.motor("Drive Motor " + mod.moduleNumber)
                                    .voltage(Units.Volts.of(mod.getDriveVoltage()))
                                    .linearPosition(Units.Meters.of(mod.getPosition().distanceMeters))
                                    .linearVelocity(Units.MetersPerSecond.of(mod.getState().speedMetersPerSecond))
                                    .linearAcceleration(
                                        Units.MetersPerSecondPerSecond.of(mod.getDriveAcceleration()));
                            }
                        },
                        this));

        sysIDChooser = new SendableChooser<>();
        sysIDChooser.setDefaultOption("Quasistatic Forward", sysIDDriveRoutine.quasistatic(Direction.kForward));
        sysIDChooser.addOption("Quasistatic Reverse", sysIDDriveRoutine.quasistatic(Direction.kReverse));
        sysIDChooser.addOption("Dynamic Forward", sysIDDriveRoutine.dynamic(Direction.kForward));
        sysIDChooser.addOption("Dynamic Reverse", sysIDDriveRoutine.dynamic(Direction.kReverse));

        enableContinuousHeadingInput();
    }

    /**
     * Drive field-relative or robot-relative.
     *
     * @param transform       X/Y velocity (m/s) and rotational velocity (rad/s)
     * @param isOpenLoop      True for open-loop drive control
     * @param isFieldRelative True for field-relative driving
     */
    public void drive(Transform2d transform, boolean isOpenLoop, boolean isFieldRelative) {
        Rotation2d rotationWithOffset = getHeading().minus(fieldOrientedOffset);

        double rotRad = rotationOverride != null
                ? rotationOverride.getAsDouble()
                : transform.getRotation().getRadians();

        SwerveModuleState[] states = Constants.DriveConstants.kSwerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                        isFieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(transform.getX(), transform.getY(), rotRad,
                                        rotationWithOffset)
                                : new ChassisSpeeds(transform.getX(), transform.getY(), rotRad),
                        0.02));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.MAX_LINEAR_VELOCITY);
        runSetpoints(states, isOpenLoop);
    }

    public void updateOdometry() {
        m_odometry.update(getHeading(), getModulePositions());

        // Generic vision update — add limelight names as needed
        for (String limelightName : new String[] { "limelight" }) { // TODO: Need to find limelight names
            LimelightHelpers.SetRobotOrientation(limelightName, getHeading().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate estimate = LimelightHelpers
                    .getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

            if (estimate != null && estimate.tagCount > 0) {
                Translation3d tagPos = LimelightHelpers
                        .getTargetPose3d_RobotSpace(limelightName).getTranslation();

                // Only trust vision within 3.5m of the target tag
                if (Math.hypot(tagPos.getX(), tagPos.getZ()) <= 3.5) {
                    m_odometry.addVisionMeasurement(
                            new Pose2d(estimate.pose.getX(), estimate.pose.getY(), getHeading()),
                            estimate.timestampSeconds);
                }
            }
        }
    }

    @AutoLogOutput(key = "Odometry/Field Position")
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public void addField2dTrajectory(List<Pose2d> poses, String name) {
        m_field2d.getObject(name).setPoses(poses);
    }

    public void addField2dPose(Pose2d pose, String name) {
        m_field2d.getObject(name).setPose(pose);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @AutoLogOutput(key = "Drivetrain/Actual Module States")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    @AutoLogOutput(key = "Drivetrain/Target Module States")
    public SwerveModuleState[] getTargetStates() {
        SwerveModuleState[] targetStates = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            targetStates[mod.moduleNumber] = mod.getTargetState();
        }
        return targetStates;
    }

    public double[] getAbsoluteAngles() {
        double[] angles = new double[4];
        for (SwerveModule mod : swerveMods) {
            angles[mod.moduleNumber] = mod.getAbsoluteAngle().getDegrees();
        }
        return angles;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void resetOdometry(Pose2d position) {
        m_odometry.resetPosition(getHeading(), getModulePositions(), position);
    }

    public void resetRotation(Rotation2d rotation) {
        resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), rotation));
    }

    public void resetToAprilTagRotation() {
        resetRotation(LimelightHelpers.getBotPose2d_wpiBlue("limelight").getRotation());
    }

    public void resetFieldOrientedHeading() {
        fieldOrientedOffset = getHeading().minus(Rotation2d.fromDegrees(180));
        resetHeading(Rotation2d.fromDegrees(
                DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 180));
    }

    public void reverseFieldOrientedHeading() {
        fieldOrientedOffset = getHeading();
        resetHeading(Rotation2d.fromDegrees(
                DriverStation.getAlliance().get() == Alliance.Blue ? 180 : 0));
    }

    @AutoLogOutput(key = "Drivetrain/Heading")
    public Rotation2d getHeading() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromDegrees(
                    MathUtil.inputModulus(gyroYawSignal.getValue().in(Units.Degrees) - 180, 0, 360));
        } else {
            SwerveModulePosition[] modulePositions = getModulePositions();
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

            for (SwerveModule mod : swerveMods) {
                moduleDeltas[mod.moduleNumber] = new SwerveModulePosition(
                        modulePositions[mod.moduleNumber].distanceMeters
                                - lastModulePositions[mod.moduleNumber].distanceMeters,
                        modulePositions[mod.moduleNumber].angle);
                lastModulePositions[mod.moduleNumber] = modulePositions[mod.moduleNumber];
            }

            simHeading = simHeading.plus(
                    new Rotation2d(Constants.DriveConstants.kSwerveKinematics.toTwist2d(moduleDeltas).dtheta));
            return simHeading;
        }
    }

    public Rotation2d getRotationalVelocity() {
        return Rotation2d.fromDegrees(gyroYawVelocitySignal.getValue().in(Units.DegreesPerSecond));
    }

    public double getPitch() {
        return m_gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return m_gyro.getRoll().getValueAsDouble();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return Constants.DriveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drivetrain/Speed")
    public double getSpeed() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public void resetHeading(Rotation2d angle) {
        m_gyro.setYaw(angle.getMeasure());
    }

    // TODO: Autoalign
    private final PIDController headingPID = new PIDController(
            Constants.AlignTargets.HEADING_KP,
            Constants.AlignTargets.HEADING_KI,
            Constants.AlignTargets.HEADING_KD);

    public Translation2d getCompensatedTarget(Translation2d realTarget) {
        double dist = getDistanceTo(realTarget);
        double tof = Constants.ShooterConstants.SHOOTER_MAP.get(dist).tof();

        ChassisSpeeds fieldSpeeds = getFieldRelativeSpeeds();

        return new Translation2d(
                realTarget.getX() - fieldSpeeds.vxMetersPerSecond * tof,
                realTarget.getY() - fieldSpeeds.vyMetersPerSecond * tof);
    }

    public void enableContinuousHeadingInput() {
        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(Constants.AlignTargets.HEADING_TOLERANCE_DEG));
    }

    public Rotation2d getTargetHeading(Translation2d target) {
        Translation2d robotPos = getPose().getTranslation();
        return Rotation2d.fromRadians(Math.atan2(target.getY() - robotPos.getY(), target.getX() - robotPos.getX()));
    }

    public double getHeadingPIDOutput(Translation2d target) {
        return headingPID.calculate(getHeading().getRadians(), getTargetHeading(target).getRadians());
    }

    public boolean atTargetHeading() {
        return headingPID.atSetpoint();
    }

    public boolean atTargetPose() {
        return headingPID.atSetpoint();
    }

    public double getDistanceTo(Translation2d target) {
        return getPose().getTranslation().getDistance(target);
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getHeading());
    }

    public void setRotationOverride(DoubleSupplier override) {
        rotationOverride = override;
    }

    public void clearRotationOverride() {
        rotationOverride = null;
    }

    public void runChassisSpeeds(ChassisSpeeds speeds) {
        runSetpoints(
                Constants.DriveConstants.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(speeds, 0.02)),
                false);
    }

    public void runSetpoints(SwerveModuleState[] states, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.MAX_LINEAR_VELOCITY);
        for (SwerveModule mod : swerveMods) {
            mod.runSetpoint(states[mod.moduleNumber], isOpenLoop);
        }
    }

    public void runDutyCycle(double drivePercentOutput, double anglePercentOutput) {
        for (SwerveModule mod : swerveMods) {
            mod.runDutyCycle(drivePercentOutput, anglePercentOutput);
        }
    }

    public double[] getDriveOutputCurrents() {
        double[] currents = new double[4];
        for (SwerveModule mod : swerveMods) {
            currents[mod.moduleNumber] = mod.getDriveOutputCurrent();
        }
        return currents;
    }

    public Command getSysIDDriveRoutine() {
        return sysIDChooser.getSelected();
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : swerveMods) {
            mod.updateInputs();
        }

        updateOdometry();
        m_field2d.setRobotPose(getPose());

        BaseStatusSignal.refreshAll(gyroYawSignal, gyroYawVelocitySignal);

        Logger.recordOutput("Drivetrain/Current Command",
                getCurrentCommand() == null ? "Nothing" : getCurrentCommand().getName());

        SmartDashboard.putNumber("Drivetrain/DistanceToHub",
                getDistanceTo(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        ? Constants.AlignTargets.BLUE_HUB
                        : Constants.AlignTargets.RED_HUB));
    }
}