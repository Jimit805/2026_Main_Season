// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.commands.Indexer.SpinStageOneManual;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.commands.Indexer.SpinStageTwoManual;
import frc.robot.commands.Intake.MoveIntakeManual;
import frc.robot.commands.Intake.RotateIntake;
import frc.robot.commands.Intake.RotateIntakeManual;
import frc.robot.commands.shooter.ManualShoot;
import frc.robot.commands.shooter.ReadyShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

import java.util.function.ObjIntConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final Intake m_Intake =  new Intake();

  private final MoveIntakeManual m_MoveIntakeManual = new MoveIntakeManual(m_Intake, m_driverController);
  private final RotateIntake m_RotateIntakeManual = new RotateIntake(m_Intake, 0.25);
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final Shooter m_Shooter = new Shooter();

  private final ManualShoot m_ManualShoot = new ManualShoot(m_Shooter, m_driverController);
  private final ReadyShooter m_ReadyShooter = new ReadyShooter(m_Shooter);
  private final Shoot m_Shoot = new Shoot(m_Shooter);

  // The driver's controller

  Trigger m_ResetEncoderFieldRelative = new JoystickButton(m_driverController, 8);
  Trigger m_TestIntakeRotation = new JoystickButton(m_driverController, 1);


  private final Indexer m_Indexer = new Indexer();
  
  // TODO : POSITIVE IS CORRECT WAY
  private final SpinStageOneManual m_IndexerStageOneManual = new SpinStageOneManual(m_Indexer, m_driverController);
  private final SpinStageTwoManual m_IndexerStageTwoManual = new SpinStageTwoManual(m_Indexer, m_driverController); // TODO change axis

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Trigger preFire = new Trigger(() -> m_Shooter.isHubAlmostActive());
    preFire.onTrue(m_ReadyShooter);

    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
   m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
          new RunCommand(  
            () -> m_robotDrive.drive(
              MathUtil.applyDeadband(m_driverController.getRawAxis(1), OIConstants.kDriveDeadband), //drive
              MathUtil.applyDeadband(m_driverController.getRawAxis(0), OIConstants.kDriveDeadband),
              MathUtil.applyDeadband(m_driverController.getRawAxis(4), OIConstants.kDriveDeadband), //rotation
                true),
            m_robotDrive));


            

  //new RunCommand( () -> m_robotDrive.setDutyCycle(0,.1),
  //    m_robotDrive));

    //m_Shooter.setDefaultCommand(m_Shoot);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    frc.robot.Button.rightTriggerDriver.whileTrue(m_Shoot);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // TODO: get actual automomous command to return
  //  return null;
 // }
}
