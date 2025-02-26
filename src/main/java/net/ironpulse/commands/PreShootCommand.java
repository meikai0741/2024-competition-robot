package net.ironpulse.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import net.ironpulse.Constants;
import net.ironpulse.drivers.Limelight;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.utils.ShootingParameters;
import net.ironpulse.utils.ShootingParametersTable;

import static edu.wpi.first.units.Units.Volts;
import static net.ironpulse.Constants.ShooterConstants.*;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Measure<Voltage> defaultVoltage = shortShootVoltage;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        defaultVoltage = shortShootVoltage;
    }

    @Override
    public void execute() {
        
        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) {
            shooterSubsystem.getIo().setShooterVoltage(defaultVoltage);
            return;
        }

        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());

        ShootingParameters parameter  = ShootingParametersTable.getInstance().getParameters(distance);
        SmartDashboard.putNumber("shooter desired angle", Units.degreesToRadians(
            shooterSubsystem.getInputs().leftShooterVelocity.magnitude()
        ));

        shooterSubsystem.getIo().setShooterVoltage(Volts.of(parameter.getVoltage()));
        // if (distance >= shortShootMaxDistance.magnitude() + 0.5) {
        //     defaultVoltage = farShootVoltage;
        //     shooterSubsystem.getIo().setShooterVoltage(farShootVoltage);
        //     return;
        // }
        // if (shortShootMaxDistance.magnitude() - 0.1 < distance && distance < shortShootMaxDistance.magnitude() + 0.5) {
        //     // looks advanced! do not touch!
        //     double tempd = (distance - shortShootMaxDistance.magnitude() + 0.1) / 0.6;
        //     tempd = tempd * (farShootVoltage.magnitude() - shortShootVoltage.magnitude());
        //     defaultVoltage = Volts.of(tempd + shortShootVoltage.magnitude());
        //     ShootingParameters parameter = ShootingParametersTable.getInstance().getParameters(distance);
        //     shooterSubsystem.getIo().setShooterVoltage(Volts.of(parameter.getVoltage()));
           
        //     // Basic math, Watson.
        //     // Method to derive:
        //     // delta (farShoot-shortShoot) / delta distance => k
        //     // substitute one point in => b
        //     // defaultVoltage = Volts.of(
        //     //         15 * distance - 29.5
        //     // ).negate();
        //     // shooterSubsystem.getIo().setShooterVoltage(Volts.of(
        //     //         15 * distance - 29.5
        //     // ).negate());
        //     return;
        // }
        // defaultVoltage = shortShootVoltage;
        // shooterSubsystem.getIo().setShooterVoltage(shortShootVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setShooterVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
