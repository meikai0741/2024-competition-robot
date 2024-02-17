package net.ironpulse.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import net.ironpulse.Constants;

import java.util.Queue;

import static edu.wpi.first.units.Units.*;
import static net.ironpulse.Constants.SwerveConstants.*;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;
    private boolean inverted;

    public ModuleIOTalonFX(int index) {
        switch (index) {
            case 0:
                driveTalon = new TalonFX(Constants.SwerveConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                turnTalon = new TalonFX(Constants.SwerveConstants.FRONT_LEFT_STEER_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                cancoder = new CANcoder(Constants.SwerveConstants.FRONT_LEFT_ENCODER_ID,
                        Constants.CAN_BUS_NAME);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.FRONT_LEFT_ENCODER_OFFSET); // MUST BE CALIBRATED
                break;
            case 1:
                driveTalon = new TalonFX(Constants.SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                turnTalon = new TalonFX(Constants.SwerveConstants.FRONT_RIGHT_STEER_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                cancoder = new CANcoder(Constants.SwerveConstants.FRONT_RIGHT_ENCODER_ID,
                        Constants.CAN_BUS_NAME);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.FRONT_RIGHT_ENCODER_OFFSET); // MUST BE CALIBRATED
                break;
            case 2:
                driveTalon = new TalonFX(Constants.SwerveConstants.BACK_LEFT_DRIVE_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                turnTalon = new TalonFX(Constants.SwerveConstants.BACK_LEFT_STEER_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                cancoder = new CANcoder(Constants.SwerveConstants.BACK_LEFT_ENCODER_ID,
                        Constants.CAN_BUS_NAME);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.BACK_LEFT_ENCODER_OFFSET); // MUST BE CALIBRATED
                break;
            case 3:
                driveTalon = new TalonFX(Constants.SwerveConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                turnTalon = new TalonFX(Constants.SwerveConstants.BACK_RIGHT_STEER_MOTOR_ID,
                        Constants.CAN_BUS_NAME);
                cancoder = new CANcoder(Constants.SwerveConstants.BACK_RIGHT_ENCODER_ID,
                        Constants.CAN_BUS_NAME);
                absoluteEncoderOffset = new Rotation2d(Constants.SwerveConstants.BACK_RIGHT_ENCODER_OFFSET); // MUST BE CALIBRATED
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        inverted = index == 1 || index == 3;
        driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);
        driveTalon.optimizeBusUtilization();
        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);

        inputs.drivePosition =
                Radians.of(Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO);
        inputs.driveVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO);
        inputs.driveAppliedVoltage = Volts.of(driveAppliedVolts.getValueAsDouble());
        inputs.driveCurrentAmps = new double[]{driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition =
                Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                        .minus(absoluteEncoderOffset);
        inputs.turnPosition =
                Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
        inputs.turnVelocity =
                RadiansPerSecond.of(Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO);
        inputs.turnAppliedVoltage = Volts.of(turnAppliedVolts.getValueAsDouble());
        inputs.turnCurrentAmps = new double[]{turnCurrent.getValueAsDouble()};

        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
                drivePositionQueue.stream()
                        .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                        .toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream()
                        .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
                        .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted =
                isTurnMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(config);
    }
}