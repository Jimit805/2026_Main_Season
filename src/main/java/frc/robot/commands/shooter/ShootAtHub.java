package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShootAtHub extends Command {

    private Shooter m_Shooter;
    private Indexer m_Indexer;

    private enum State { PRESHOOT, SHOOTING }
    private State state;

    public ShootAtHub(Shooter shooter, Indexer indexer) {
        m_Shooter = shooter;
        m_Indexer = indexer;
        addRequirements(m_Shooter, m_Indexer);
    }

    @Override
    public void initialize() {
        state = State.PRESHOOT;
        m_Shooter.spinFlywheels(m_Shooter.getTargetVelocity());
    }

    @Override
    public void execute() {
        if (m_Shooter.atTargetSpeed() && m_Shooter.atTargetPosition()) {
            state = State.SHOOTING;
        } else {
            state = State.PRESHOOT;
        }

        switch (state) {
            case PRESHOOT:
                m_Shooter.spinFlywheels(m_Shooter.getTargetVelocity());
                m_Shooter.pivotShooter(m_Shooter.getTargetPosition());
                break;
            case SHOOTING:
                m_Indexer.runStageOneMotor(Constants.IndexerConstants.STAGE_ONE_INTAKE_SPEED);
                m_Indexer.runStageTwoMotor(Constants.IndexerConstants.STAGE_TWO_INTAKE_SPEED);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Shooter.spinFlywheels(0);
        m_Shooter.pivotShooter(Constants.ShooterConstants.SHOOTER_STOW);
        m_Indexer.stopAll();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
