// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends frc.slicelibs.TalonFXPositionalSubsystem {

  private TalonFX rotationMotor;
  private boolean active; // Tells whether or not intake is active
  // TODO maybe change to a method?


  /** Creates a new Intake. */
  public Intake() {
    super(
      new int[] { Constants.IntakeConstants.EXTENDER_MOTOR_ID },
      new boolean[] { false },
      Constants.IntakeConstants.EXTENDER_KP, Constants.IntakeConstants.EXTENDER_KI, Constants.IntakeConstants.EXTENDER_KP, Constants.IntakeConstants.EXTENDER_KG,
      Constants.IntakeConstants.EXTENDER_RATIO,
      GravityTypeValue.Elevator_Static,
      Constants.IntakeConstants.POSITION_CONVERSION_FACTOR,
      Constants.IntakeConstants.VELOCITY_CONVERSION_FACTOR,
      Constants.CTRE_CONFIGS.m_intakeConfigs
    );
    setEncoderPosition(0);
    rotationMotor = new TalonFX(Constants.IntakeConstants.ROTATION_MOTOR_ID);
  }

  /**
   * Changes if intake is active
   * @param setTrue true or false value to set boolean
  */
  public void setActive(boolean setTrue) {
    active = setTrue;
  }
  
  /**
   * Returns true if boolean is active
   * @return
   */
  public boolean isActive() {
    return active;
  }
  
  /**
   * Sets speed of the extender motor
   * @param speed speed to set the motor to (-1.0 to 1.0)
   */
  public void moveExtendMotor(double speed) {
    set(speed);
  }

  /**
   * Sets speed of rotation motor
   * @param speed speed to set the motor to (-1.0 to 1.0)
   */
  public void moveRotationMotor(double speed) {
    rotationMotor.set(speed);
  }

  /**
   * Moves the intake to the set position
   * @param position position to have the intake move to
   */
  public void moveIntakeToPosition(double position) {
    setPosition(position);
  }

  /**
   * Increases motor speed
   * @param speed speed to set the motor to (-1.0 to 1.0)
   */
  public void speedMotorUp(double speed){
      
    VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
    rotationMotor.setControl(request.withVelocity(speed));
  }
  
  public double getExtenderPosition(){
    return getPositions()[0];
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake extender position: ", getExtenderPosition());
    // This method will be called once per scheduler run
  }
}
