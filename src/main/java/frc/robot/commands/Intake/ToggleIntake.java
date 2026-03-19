package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class ToggleIntake extends Command {

    private final Intake m_intake;
    private final Indexer m_indexer;

    public ToggleIntake(Intake intake, Indexer indexer) {
        m_intake = intake;
        m_indexer = indexer;
        addRequirements(m_intake, m_indexer);
    }

    @Override
    public void initialize() {
        m_intake.moveIntakeToPosition(Constants.IntakeConstants.DEPLOYED_POSITION);
        m_intake.spinRoller(Constants.IntakeConstants.ROLLER_SPEED);
        m_indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        // Retract when cancelled (second button press) or interrupted
        m_intake.moveIntakeToPosition(Constants.IntakeConstants.STOWED_POSITION);
        m_intake.stopRoller();
        m_indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until toggleOnTrue cancels it
    }
}