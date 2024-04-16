package net.ironpulse.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import net.ironpulse.Constants;
import net.ironpulse.drivers.LimelightHelpers;
import net.ironpulse.subsystems.indicator.IndicatorIO;
import net.ironpulse.subsystems.indicator.IndicatorSubsystem;
import net.ironpulse.subsystems.shooter.ShooterSubsystem;
import net.ironpulse.subsystems.swerve.FieldCentricHeading;
import net.ironpulse.subsystems.swerve.FieldCentricTargetHeading;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;
import net.ironpulse.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static net.ironpulse.Constants.Logger.debug;
import static net.ironpulse.Constants.SwerveConstants.*;

public class LongShotCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;
    private final FieldCentricHeading drive = new FieldCentricHeading()
            .withDeadband(maxSpeed.magnitude() * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
    private final Translation2d targetTranslation = new Translation2d(2, 2); // modify this

    public LongShotCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            SwerveSubsystem swerveSubsystem,
            CommandXboxController driverController) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        drive.HeadingController.setPID(headingGains.kP, headingGains.kI, headingGains.kD);
        drive.HeadingController.enableContinuousInput(0, 360.0);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
        Rotation2d targetAngle = Rotation2d.fromDegrees(135); // test this first
        // Translation2d currentTranslation =
        // swerveSubsystem.getPose().getTranslation();
        // Translation2d deltaTranslation = targetTranslation.minus(currentTranslation);
        // Rotation2d targetAngle = deltaTranslation.getAngle();

        swerveSubsystem.applyRequest(() -> drive
                .withVelocityX(Utils.sign(-driverController.getLeftY())
                        * xLimiter.calculate(Math.abs(driverController.getLeftY()))
                        * maxSpeed.magnitude())
                .withVelocityY(
                        Utils.sign(-driverController.getLeftX()) * maxSpeed.magnitude()
                                * yLimiter.calculate(Math.abs(driverController.getLeftX())))
                .withCurrentAngle(swerveSubsystem.getPose().getRotation())
                .withTargetAngle(targetAngle))
                .execute();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }
}
