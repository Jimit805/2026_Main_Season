// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.ShooterCalculations;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Shooter extends SubsystemBase {

  private TalonFX pivotMotor, leftShooterMotor, rightShooterMotor;
  //private Encoder pivotEncoder, leftEncoder, rightEncoder;

  private double targetSpeed, targetPosition;

  /** Creates a new Shooter. */
  public Shooter() {
    // Define the motors for pivoting the shooter and the flywheels.
    pivotMotor = new TalonFX(Constants.ShooterConstants.PIVOT_MOTOR_ID);
    leftShooterMotor = new TalonFX(Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID);
    rightShooterMotor = new TalonFX(Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID);

    // Set the motor configs.
    leftShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);
    rightShooterMotor.getConfigurator().apply(Constants.CTRE_CONFIGS.shooterConfigs);

    System.out.println("Pro License: " + leftShooterMotor.getIsProLicensed());
  }

  /**
   * Sets the flywheels to spin at a certain speed.
   * @param speed The speed to set (a value between -1.0 and 1.0)
   */
  public void spinFlywheels(double speed){
    leftShooterMotor.set(speed);
    //rightShooterMotor.set(speed);
  }

  /**
   * Sets the pivot to a certain speed.
   * @param speed The speed to set (a value between -1.0 and 1.0)
   */
  public void pivotShooter(double speed){
    pivotMotor.set(speed);
  }

  /**
   * Accelerates the flywheels to a certain speed.
   * @param speed The speed to spin at in rotations per second.
   */
  public void speedUpFlywheels(double speed){
    targetSpeed = speed;
    
    VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    leftShooterMotor.setControl(request.withVelocity(speed));
    rightShooterMotor.setControl(request.withVelocity(speed));

  }

  /**
   * Set the shooter's pivot motor to a specific angle.
   * @param position The position to set in rotations.
   */
  public void pivotShooterToPosition(double position){
    targetPosition = position;

    PositionVoltage request = new PositionVoltage(position); 

    pivotMotor.setControl(request.withPosition(position)); 

  }

  /**
   * Check if the motor is at a specific speed.
   * @param error How much error is allowed to be considered "at speed."
   * @return Returns whether or not the motor is at speed.
   */
  public boolean atTargetSpeed(double error){
    double currentSpeed = leftShooterMotor.getVelocity().getValueAsDouble();
    currentSpeed += rightShooterMotor.getVelocity().getValueAsDouble() / 2;
    if(Math.abs(targetSpeed - currentSpeed) >= error){
      return true;
    }
    return false;
  }

  /**
   * Determines if the hub is active (from WPILib website)
   * @return Returns if the hub is active or not
   */
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  /**
   * Determines if the hub is 4 seconds from active (from WPILib website)
   * @return Returns if the hub is 4 seconds from active or not
   */
  public boolean isHubAlmostActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > (130 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > (105 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > (80 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > (55 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > (30 - Constants.FieldConstants.SPEED_SHOOTER_AT)) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
