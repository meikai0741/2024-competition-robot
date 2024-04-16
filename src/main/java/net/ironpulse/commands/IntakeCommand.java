package net.ironpulse.commands;

import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.subsystems.beambreak.BeamBreakSubsystem;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.intaker.IntakerSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public IntakeCommand(
            IntakerSubsystem intakerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            ShooterSubsystem shooterSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem.getIo()
                .setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
        shooterSubsystem.getIo().setShooterVoltage(Volts.of(1));
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIo()
                .setIntakeVoltage(Volts.zero());
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
        if (interrupted) return;
        indicatorSubsystem
                .setPattern(IndicatorIO.Patterns.FINISH_INTAKE);
    }


    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn;
    }
}
