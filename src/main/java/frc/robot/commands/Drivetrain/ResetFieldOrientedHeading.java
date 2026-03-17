package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class ResetFieldOrientedHeading extends Command {

    private final Drivetrain m_drivetrain;

    public ResetFieldOrientedHeading(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        m_drivetrain.resetFieldOrientedHeading();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}