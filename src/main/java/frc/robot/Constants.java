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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
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
    public static final Rotation2d kFrontLeftAngulatOffset = Rotation2d.fromDegrees(50/*13.59*/);
    public static final Rotation2d kFrontRightAngularOffset = Rotation2d.fromDegrees(342.31);
    public static final Rotation2d kRearLeftAngularOffset = Rotation2d.fromDegrees(113.34);
    public static final Rotation2d kRearRightAngularOffset = Rotation2d.fromDegrees(95.34);

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
    public static final int FRONT_LEFT_ENCODER_ID = 0;
    public static final int FRONT_RIGHT_ENCODER_ID = 1;
    public static final int BACK_LEFT_ENCODER_ID = 3;
    public static final int BACK_RIGHT_ENCODER_ID = 2;


    //TODO: Figure out PIDs for both Drive Motors and Turn Motors
    public static final double DRIVE_KP = .05;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;

    public static final double TURN_KP = .05;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;
    

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

}
