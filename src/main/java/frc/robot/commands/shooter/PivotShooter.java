// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.io.Console;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShooterCalculations;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotShooter extends Command {
  private Shooter mainShooter;
  double[] result = new double[2];

  Thread calcThread;
  Runnable calcRunnable;
  // private double rotations;
  /** Creates a new PivotShooter. */
  public PivotShooter(Shooter m_shooter) {
    mainShooter = m_shooter;
    calcRunnable = () -> {  
      double distance = ShooterCalculations.distanceToHub();
      try {
        result = ShooterCalculations.calculateShooterTrajectory(distance);
        mainShooter.pivotShooterToPosition(result[0] * 2 * Math.PI);
       } catch (IndexOutOfBoundsException e) {
        result[0] = 0;
        result[1] = 0;
      }
    };
    // rotations = targetAngle * 2 * Math.PI;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (calcThread == null || !calcThread.isAlive()) {
      calcThread = new Thread(calcRunnable);
      calcThread.start();
    }
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
