// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Indexer.SpinStageTwo;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndIndex extends ParallelCommandGroup {
  private Shoot shoot;
  private Shooter shooter;
  private Indexer indexer;
  /** Creates a new ShootAndIndex. */
  public ShootAndIndex(Shooter shooter, Indexer indexer) {
    this.shooter = shooter;
    this.indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // TODO tune Stage Two Speeds
    addCommands(new SpinStageTwo(indexer, 0.3), new Shoot(shooter));
  }
}
