package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import frc.slicelibs.configs.CTREConfigs;

/**
 * Robot-wide constants. Hardware configurations for the swerve drivetrain
 * (motor IDs, encoder offsets, PID gains) now live in
 * {@link frc.robot.generated.TunerConstants}.
 */
public final class Constants {

    public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();

    public final class IndexerConstants {
        public static final int STAGE_ONE_MOTOR_ID = 18;
        public static final int STAGE_TWO_MOTOR_ID = 1;

        public static final double STAGE_ONE_INTAKE_SPEED = 0.5;
        public static final double STAGE_TWO_INTAKE_SPEED = 0.5;

        public static final int INDEXER_STATOR_CURRENT_LIMIT = 40;
        public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 30;
    }

    public final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public final class AlignTargets {
        public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034);
        public static final Translation2d RED_HUB = new Translation2d(11.915, 4.034);

        public static final Translation2d BLUE_PASS_LEFT = new Translation2d(3.0, 6.5);
        public static final Translation2d BLUE_PASS_RIGHT = new Translation2d(3.0, 1.5);
        public static final Translation2d RED_PASS_LEFT = new Translation2d(13.5, 6.5);
        public static final Translation2d RED_PASS_RIGHT = new Translation2d(13.5, 1.5);

        public static final double HEADING_KP = 6.0;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.3;
        public static final double HEADING_TOLERANCE_DEG = 2.0;
    }

    public final class IntakeConstants {
        public static final int ROTATION_MOTOR_ID = 6;
        public static final int EXTENDER_MOTOR_ID = 5;

        public static final double EXTENDER_KP = 15.0;
        public static final double EXTENDER_KI = 0.0;
        public static final double EXTENDER_KD = 0.0;
        public static final double EXTENDER_KG = 0.0;
        public static final double EXTENDER_RATIO = 50.0 / 9.0;
        public static final int EXTENDER_STATOR_CURRENT_LIMIT = 60;
        public static final int EXTENDER_SUPPLY_CURRENT_LIMIT = 40;
        public static final double POSITION_CONVERSION_FACTOR = (Units.inchesToMeters(1.0) * Math.PI) / EXTENDER_RATIO;
        public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;

        public static final double STOWED_POSITION = Units.inchesToMeters(0.0);
        public static final double DEPLOYED_POSITION = Units.inchesToMeters(11.5);
        public static final double BUMPER_WIDTH = Units.inchesToMeters(3.5);
        public static final double OSCILLATION_AMOUNT = Units.inchesToMeters(0.75);
        public static final double OSCILLATION_DIFF = Units.inchesToMeters(0.25);
        public static final double ROLLER_SPEED = 0.8;
        public static final double ROLLER_RETRACT_SPEED = 0.0;
        public static final double ROLLER_GEAR_RATIO = 2.0;
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 40;
        public static final int ROLLER_SUPPLY_CURRENT_LIMIT = 30;
    }

    public final class ShooterConstants {
        public static final int PIVOT_MOTOR_ID = 4;
        public static final int LEFT_SHOOTER_MOTOR_ID = 3;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 2;

        public static final double FLYWHEEL_KP = 0.35;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KS = 0.0;
        public static final double FLYWHEEL_KV = 0.12;
        public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80;
        public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60;

        public static final double FLYWHEEL_GEAR_RATIO = 1.4;
        public static final double PIVOT_GEAR_RATIO = 4.75 * 16.5;
        public static final double POSITION_CONVERSION_FACTOR = 1.0 / PIVOT_GEAR_RATIO;
        public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;

        public static final double AIM_KP = 0.05;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.0;
        public static final double AIM_KG = 0.01;
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 60;
        public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 40;

        public static final double SHOOTER_STOW = 12.0;
        public static final double FLYWHEEL_RPM_ACCEPTABLE_ERROR = 10.0;
        public static final double VERTICAL_AIM_ACCEPTABLE_ERROR = .1;

        public static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
                MathUtil::inverseInterpolate, FullShooterParams::interpolate);
        static {
            SHOOTER_MAP.put(0.5, new FullShooterParams(2800.0, 12.0, 0.38));
            SHOOTER_MAP.put(1.0, new FullShooterParams(3100.0, 14.0, 0.45));
            SHOOTER_MAP.put(1.5, new FullShooterParams(3400.0, 17.0, 0.52));
            SHOOTER_MAP.put(2.0, new FullShooterParams(3650.0, 20.0, 0.60));
            SHOOTER_MAP.put(2.5, new FullShooterParams(3900.0, 23.0, 0.68));
            SHOOTER_MAP.put(3.0, new FullShooterParams(4100.0, 24.0, 0.76));
            SHOOTER_MAP.put(3.5, new FullShooterParams(4350.0, 26.0, 0.85));
            SHOOTER_MAP.put(4.0, new FullShooterParams(4550.0, 29.0, 0.94));
            SHOOTER_MAP.put(4.5, new FullShooterParams(4700.0, 31.0, 1.00));
            SHOOTER_MAP.put(5.0, new FullShooterParams(5000.0, 33.0, 1.10));
        }

        public record FullShooterParams(double rpm, double hoodAngle, double tof)
                implements Interpolatable<FullShooterParams> {
            @Override
            public FullShooterParams interpolate(FullShooterParams endValue, double t) {
                return new FullShooterParams(
                        rpm + (endValue.rpm - rpm) * t,
                        hoodAngle + (endValue.hoodAngle - hoodAngle) * t,
                        tof + (endValue.tof - tof) * t);
            }
        }

        public static final double SHOOTER_HEIGHT = 1.7891;
        public static final double FLYWHEEL_RADIUS = 0.1667;
        public static final double LIMELIGHT_ANGLE = 72.5;
        public static final double LIMELIGHT_HEIGHT = 1.525;
    }

    public final class FieldConstants {
        public static final double GRAVITY = 32.185;
        public static final double HUB_HEIGHT = 6.15;
        public static final double HUB_HALF_LENGTH = 1.958335;
        public static final double HUB_APRILTAG_HEIGHT = Units.inchesToMeters(44.25);
        public static final double SPEED_SHOOTER_AT = 4;
    }
}
