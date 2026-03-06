// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public static final Translation2d BLUE_HUB = new Translation2d(4.62534, 4.03479); // TODO: Check the hubs (in metres)
    public static final Translation2d RED_HUB = new Translation2d(11.91514, 4.03479);
    public static final double GRAVITY = 32.185; // Feet per second per second

    // Height and length of the hub
    public static final double HUB_HEIGHT = 6; // Feet 
    public static final double HUB_HALF_LENGTH = 1.958335; // Feet    

    // How long to speed up shooter before hub active
    public static final double SPEED_SHOOTER_AT = 4; // Seconds

  }

}
