// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.slicelibs.configs.CTREConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final CTREConfigs CTRE_CONFIGS = new CTREConfigs();


  public static class IndexerConstants {
    //TODO get motor IDs
    public static final int STAGE_ONE_MOTOR_ID = 18;
    public static final int STAGE_TWO_MOTOR_ID = 1;
  }

  public static class DriveConstants {
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(15.4498); //TODO: fine tune measurements
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.1782);  //TODO: fine tune measurements

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 3 / Math.hypot(kTrackWidth / 2, kWheelBase / 2); // radians per second

    public static final double kWheelRadius = 0.0508; //in Meters

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));  //back left

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = Math.PI / 2;

    //Angular offsets of the wheels
    //TODO find actual angle offsets
    public static final Rotation2d kFrontLeftAngulatOffset = Rotation2d.fromDegrees(293.37);
    public static final Rotation2d kFrontRightAngularOffset = Rotation2d.fromDegrees(174.96);
    public static final Rotation2d kRearLeftAngularOffset = Rotation2d.fromDegrees(187.2);

    public static final Rotation2d kRearRightAngularOffset = Rotation2d.fromDegrees(184.428);

    // public static final Rotation2d kFrontLeftAngulatOffset = Rotation2d.fromDegrees(0);
    // public static final Rotation2d kFrontRightAngularOffset = Rotation2d.fromDegrees(0);
    // public static final Rotation2d kRearLeftAngularOffset = Rotation2d.fromDegrees(0);
    // public static final Rotation2d kRearRightAngularOffset = Rotation2d.fromDegrees(0);

    /* MOTOR IDS */
    public static final int FRONT_LEFT_DRIVE_ID = 17;
    public static final int FRONT_RIGHT_DRIVE_ID = 13;
    public static final int BACK_LEFT_DRIVE_ID = 15;
    public static final int BACK_RIGHT_DRIVE_ID = 11;

    public static final int FRONT_LEFT_TURN_ID = 16;
    public static final int FRONT_RIGHT_TURN_ID = 12;
    public static final int BACK_LEFT_TURN_ID = 14;
    public static final int BACK_RIGHT_TURN_ID = 10;
    
    /* Encoder IDs */
    // TODO GET ACTUAL CANCODER IDS
    public static final int FRONT_LEFT_ENCODER_ID = 22;
    public static final int FRONT_RIGHT_ENCODER_ID = 21;
    public static final int BACK_LEFT_ENCODER_ID = 20;
    public static final int BACK_RIGHT_ENCODER_ID = 23;

    /* Drivetrain Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = (5.79 / 1.0); // 5.79:1
    public static final double ANGLE_GEAR_RATIO = (25.0 / 1.0); // 25:1


    //TODO: Figure out PIDs for both Drive Motors and Turn Motors
    public static final double DRIVE_KP = .05;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;

    public static final double TURN_KP = 6.789;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;
    

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static class IntakeConstants {

    //TODO find actual motor IDs
    public static final int ROTATION_MOTOR_ID = 6;
    public static final int EXTENDER_MOTOR_ID = 5;

  }
  
  public static class ShooterConstants {
    
    public static final int PIVOT_MOTOR_ID = 4;
    public static final int LEFT_SHOOTER_MOTOR_ID = 3;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 2;

    //TODO: tune PIDs
    public static final double FLYWHEEL_KP = 0.05;
    public static final double FLYWHEEL_KI = 0.0;
    public static final double FLYWHEEL_KD = 0.0;

    public static final double AIM_KP = 0.05;
    public static final double AIM_KI = 0.0;
    public static final double AIM_KD = 0.0;

    public static final double SHOOTER_STOW = 0.0; // The angle at which the shooter is considered stowed

    //TODO: get actual shooter rpm
    public static final double SHOOTER_RPM = 60;

    // Robot dimensions
    public static final double SHOOTER_HEIGHT = 1.7891; // Feet
    public static final double FLYWHEEL_RADIUS = 0.1667; // Feet
    public static final double LIMELIGHT_ANGLE = 25; // TODO: Fill in degrees
    public static final double LIMELIGHT_HEIGHT = 1.5; // TODO: Fill in feet

    // Maximums and minimums (Tune these)
    public static final double MAX_FLYWHEEL_VELOCITY = 35; // Feet per second
    public static final double MIN_FLYWHEEL_VELOCITY = 10; // Feet per second
    public static final double MAX_SHOOTER_ANGLE = 1.36136; // Radians
    public static final double MIN_SHOOTER_ANGLE = 0.994838; // Radians

    // Errors //
    public static final double FLYWHEEL_RPM_ACCEPTABLE_ERROR = 2; // The maximum error allowed in the flywheel RPM
    public static final double VERTICAL_AIM_ACCEPTABLE_ERROR = 2; // The maximum error allowed in the shooter angle vertically, in degrees
    public static final double HORIZONTAL_AIM_ACCEPTABLE_ERROR = 2; // The maximum error allowed in the shooter angle horizontally (controlled by drivetrain). in degrees
    public static final double MAXIMUM_SHOOTING_DRIVETRAIN_SPEED = 0.1; // The maximum speed that the drivetrain can move at and shoot
    
    // Optimization for Maths (Tune these)
    public static final double SHOOTER_STEP = 0.1;
    public static final int SHOOTER_DIE_TIME = 1000000; // Maximum number of optimizations allowed
    public static final double H = 0.0000001;
  }

  public static class FieldConstants {
    /*public static final Translation2d BLUE_HUB = new Translation2d(4.62534, 4.03479); // TODO: Check the hubs (in metres)
    public static final Translation2d RED_HUB = new Translation2d(11.91514, 4.03479);*/
    public static final double GRAVITY = 32.185; // Feet per second per second

    // Height and length of the hub
    public static final double HUB_HEIGHT = 6.4; // Feet 
    public static final double HUB_HALF_LENGTH = 1.958335; // Feet  
    public static final double HUB_APRILTAG_HEIGHT = Units.inchesToMeters(44.25); // inches  

    // How long to speed up shooter before hub active
    public static final double SPEED_SHOOTER_AT = 4; // Seconds

  }

}
