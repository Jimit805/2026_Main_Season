// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterCalculations;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReadyShooter extends Command {
  private Shooter m_shooter;
  private double m_speed;
  double[] result = new double[2];

  /** Creates a new ReadyShooter. */
  public ReadyShooter(Shooter shooter) {
    m_shooter = shooter;
    m_speed = result[1];
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = ShooterCalculations.distanceToHub();
    try {
      result = ShooterCalculations.calculateShooterTrajectory(distance);
    } catch (IndexOutOfBoundsException e) {
      result[0] = 0;
      result[1] = 0;
    }
    m_shooter.speedUpFlywheels(m_speed); // Spin up flywheels 4 seconds before hub active
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
