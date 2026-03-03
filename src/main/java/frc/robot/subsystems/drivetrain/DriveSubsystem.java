// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeftModule = new SwerveModule(
   0,
   Constants.DriveConstants.FRONT_LEFT_DRIVE_ID, 
   Constants.DriveConstants.FRONT_LEFT_TURN_ID, 
   Constants.DriveConstants.kFrontLeftChassisAngularOffset, 
   Constants.DriveConstants.kFrontLeftAngulatOffset);

  private final SwerveModule m_frontRightModule = new SwerveModule(
    1,
    Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
    Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
    Constants.DriveConstants.kFrontRightChassisAngularOffset, 
    Constants.DriveConstants.kFrontRightAngularOffset);

  private final SwerveModule m_backLeftModule = new SwerveModule(
    2, 
    Constants.DriveConstants.BACK_LEFT_DRIVE_ID, 
    Constants.DriveConstants.BACK_LEFT_TURN_ID, 
    Constants.DriveConstants.kRearLeftChassisAngularOffset, 
    Constants.DriveConstants.kRearLeftAngularOffset);

  private final SwerveModule m_backRightModule = new SwerveModule(3, 
  Constants.DriveConstants.BACK_RIGHT_DRIVE_ID, 
  Constants.DriveConstants.BACK_RIGHT_TURN_ID, 
  Constants.DriveConstants.kRearRightChassisAngularOffset, 
  Constants.DriveConstants.kRearRightAngularOffset);

  //TODO add/fix Gyro

  private Pigeon2 m_gyro = new Pigeon2(24); 

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeftModule.getSwerveModulePosition(),
          m_frontRightModule.getSwerveModulePosition(),
          m_backLeftModule.getSwerveModulePosition(),
          m_backRightModule.getSwerveModulePosition()
      });

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeftModule.getSwerveModulePosition(),
            m_frontRightModule.getSwerveModulePosition(),
            m_backLeftModule.getSwerveModulePosition(),
            m_backRightModule.getSwerveModulePosition()
        },
        pose);
  }


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_backRightModule.setDesiredState(swerveModuleStates[2]);
    m_backLeftModule.setDesiredState(swerveModuleStates[3]);

   // System.out.println("Speed: " + m_frontLeftModule.getSwerveModuleState().speedMetersPerSecond + "Rot: " + m_backLeftModule.getSwerveModuleState().angle.getRadians());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
