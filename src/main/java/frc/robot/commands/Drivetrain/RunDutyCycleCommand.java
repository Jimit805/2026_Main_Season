package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;

/**
 * Directly sets all swerve module drive and angle motors to given percent
 * outputs.
 */
public class RunDutyCycleCommand extends Command {

    private final Drivetrain m_drivetrain;
    private final double drivePercentOutput, anglePercentOutput;

    /**
     * @param drivetrain         Drivetrain subsystem instance
     * @param drivePercentOutput Drive motor output (-1 to 1)
     * @param anglePercentOutput Angle motor output (-1 to 1)
     */
    public RunDutyCycleCommand(Drivetrain drivetrain, double drivePercentOutput, double anglePercentOutput) {
        addRequirements(drivetrain);
        m_drivetrain = drivetrain;
        this.drivePercentOutput = drivePercentOutput;
        this.anglePercentOutput = anglePercentOutput;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.runDutyCycle(drivePercentOutput, anglePercentOutput);
    }

    @Override
    public void end(boolean interruptSed) {
        m_drivetrain.runDutyCycle(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}