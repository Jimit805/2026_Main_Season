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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  
  private final SwerveModule m_frontLeftModule = new SwerveModule(
   0,
   Constants.DriveConstants.FRONT_LEFT_DRIVE_ID, 
   Constants.DriveConstants.FRONT_LEFT_TURN_ID, 
   Constants.DriveConstants.FRONT_LEFT_ENCODER_ID,
   Constants.DriveConstants.kFrontLeftChassisAngularOffset, 
   Constants.DriveConstants.kFrontLeftAngulatOffset);

  private final SwerveModule m_frontRightModule = new SwerveModule(
    1,
    Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
    Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
    Constants.DriveConstants.FRONT_RIGHT_ENCODER_ID,
    Constants.DriveConstants.kFrontRightChassisAngularOffset, 
    Constants.DriveConstants.kFrontRightAngularOffset);

  private final SwerveModule m_backLeftModule = new SwerveModule(
    2, 
    Constants.DriveConstants.BACK_LEFT_DRIVE_ID, 
    Constants.DriveConstants.BACK_LEFT_TURN_ID, 
    Constants.DriveConstants.BACK_LEFT_ENCODER_ID,
    Constants.DriveConstants.kRearLeftChassisAngularOffset, 
    Constants.DriveConstants.kRearLeftAngularOffset);

  private final SwerveModule m_backRightModule = new SwerveModule(3, 
    Constants.DriveConstants.BACK_RIGHT_DRIVE_ID, 
    Constants.DriveConstants.BACK_RIGHT_TURN_ID, 
    Constants.DriveConstants.BACK_RIGHT_ENCODER_ID,
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
    SmartDashboard.putNumber("FL Swerve State Pre-desaturate", swerveModuleStates[0].angle.getRotations());
    SmartDashboard.putNumber("FR Swerve State Pre-desaturate", swerveModuleStates[1].angle.getRotations());
    SmartDashboard.putNumber("BL Swerve State Pre-desaturate", swerveModuleStates[3].angle.getRotations());
    SmartDashboard.putNumber("BR Swerve State Pre-desaturate", swerveModuleStates[2].angle.getRotations());
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    SmartDashboard.putNumber("FL Swerve State Post-desaturate", swerveModuleStates[0].angle.getRotations());
    SmartDashboard.putNumber("FR Swerve State Post-desaturate", swerveModuleStates[1].angle.getRotations());
    SmartDashboard.putNumber("BL Swerve State Post-desaturate", swerveModuleStates[3].angle.getRotations());
    SmartDashboard.putNumber("BR Swerve State Post-desaturate", swerveModuleStates[2].angle.getRotations());
    m_frontLeftModule.setDesiredState(swerveModuleStates[0]);
    m_frontRightModule.setDesiredState(swerveModuleStates[1]);
    m_backRightModule.setDesiredState(swerveModuleStates[2]);
    m_backLeftModule.setDesiredState(swerveModuleStates[3]);
    
  }

  public void setDutyCycle (double DrivePercent, double RotationPercent) {
    m_backRightModule.setDutyCycle(DrivePercent, RotationPercent);
    m_backLeftModule.setDutyCycle(DrivePercent, RotationPercent);
    m_frontRightModule.setDutyCycle(DrivePercent, RotationPercent);
    m_frontLeftModule.setDutyCycle(DrivePercent, RotationPercent);

  }

  public void resetEncoders(){
    m_frontLeftModule.resetEncoders();
    m_backLeftModule.resetEncoders();
    m_frontRightModule.resetEncoders();
    m_backRightModule.resetEncoders();
  }

  //public void resetGyro(){
  //  m_gyro.reset();
 // }
  
  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Front Left Relative", m_frontLeftModule.getRelativeEncoder());
    SmartDashboard.putNumber("Front Right Relative", m_frontRightModule.getRelativeEncoder());
    SmartDashboard.putNumber("Back Left Relative", m_backLeftModule.getRelativeEncoder());
    SmartDashboard.putNumber("Back Right Relative", m_backRightModule.getRelativeEncoder());

    SmartDashboard.putNumber("Front Left Absolute", m_frontLeftModule.getAbsoluteEncoder());
    SmartDashboard.putNumber("Front Right Absolute", m_frontRightModule.getAbsoluteEncoder());
    SmartDashboard.putNumber("Back Left Absolute", m_backLeftModule.getAbsoluteEncoder());
    SmartDashboard.putNumber("Back Eight Absolute", m_backRightModule.getAbsoluteEncoder());

    //SmartDashboard.putNumber("Front Left Rotation : ", m_frontLeftModule.getTurnRotation());
    //SmartDashboard.putNumber("Front Right Rotation : ", m_frontRightModule.getTurnRotation());
    //SmartDashboard.putNumber("Back Left Rotation : ", m_backLeftModule.getTurnRotation());
    //SmartDashboard.putNumber("Back Right Rotation : ", m_backRightModule.getTurnRotation());
    //SmartDashboard.putNumber( "Back Left ROT: " , m_backLeftModule.getSwerveModuleState().angle.getDegrees());
    //SmartDashboard.putNumber( "Back Right ROT: " , m_backRightModule.getSwerveModuleState().angle.getDegrees());
    //SmartDashboard.putNumber( "Front Right ROT: " , m_frontRightModule.getSwerveModuleState().angle.getDegrees());
    //SmartDashboard.putNumber( "Front Left ROT: " , m_frontLeftModule.getSwerveModuleState().angle.getDegrees());
  }    
  
  }

