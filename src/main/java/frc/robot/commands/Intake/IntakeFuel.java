// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Indexer.SpinStageOne;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFuel extends ParallelCommandGroup {

  private Intake m_Intake;
  private Indexer m_Indexer;


  /** Creates a new IntakeFuel. */
  public IntakeFuel(Intake intake, Indexer indexer) {
    m_Intake = intake;
    m_Indexer = indexer;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // TODO get position for move intake
    // TODO get correct speed for rotate intake
    // TODO get correct speed for StageOne of the indexer
    addCommands(new MoveIntake(intake, 19), new RotateIntake(intake, 0.25), new SpinStageOne(indexer, 0.25));
  }
}
