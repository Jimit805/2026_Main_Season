package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class OscillateIntake extends Command {

    private final Intake m_intake;
    private double targetPosition;
    private boolean movingIn; // true = moving toward stow, false = moving back out

    public OscillateIntake(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        targetPosition = Constants.IntakeConstants.DEPLOYED_POSITION;
        movingIn = true;
    }

    @Override
    public void execute() {
        m_intake.moveIntakeToPosition(targetPosition);

        boolean atTarget = Math.abs(m_intake.getExtenderPosition() - targetPosition) < Units.inchesToMeters(0.25);

        if (atTarget) {
            if (movingIn) {
                // step back out 2 inches
                targetPosition += Units.inchesToMeters(2.0);
                movingIn = false;
            } else {
                // step in 3 inches
                targetPosition -= Units.inchesToMeters(3.0);
                movingIn = true;
            }
            // Clamp so we never go past stow
            targetPosition = Math.max(targetPosition, Constants.IntakeConstants.STOWED_POSITION);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.moveIntakeToPosition(Constants.IntakeConstants.BUMPER_WIDTH);
    }

    @Override
    public boolean isFinished() {
        return m_intake.isStowed();
    }
}