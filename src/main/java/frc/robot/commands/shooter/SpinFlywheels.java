// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinFlywheels extends Command {
  private Shooter mainShooter;
  private double angularSpeed;
  /** Creates a new SpinFlywheels. */
  public SpinFlywheels(Shooter m_shooter, double linearSpeed) {
    mainShooter = m_shooter;
    angularSpeed = linearSpeed / Constants.ShooterConstants.FLYWHEEL_RADIUS;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mainShooter.speedUpFlywheels(angularSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mainShooter.spinFlywheels(-angularSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mainShooter.spinFlywheels(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
