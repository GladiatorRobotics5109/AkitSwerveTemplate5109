package frc.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModuleIOTalonFx implements SwerveModuleIO {
    private final TalonFX m_drive;
    private final TalonFX m_turn;

    private final boolean m_useFOC;

    private final VoltageOut m_driveVoltage;
    private final VoltageOut m_turnVoltage;

    private final VelocityVoltage m_driveVelocity;
    private final PositionVoltage m_turnPosition;

    private final StatusSignal<Double> m_signalDrivePosition;
    private final StatusSignal<Double> m_signalDriveVelocity;
    private final StatusSignal<Double> m_signalDriveAcceleration;
    private final StatusSignal<Double> m_signalDriveTemp;
    private final StatusSignal<Double> m_signalDriveAppliedVoltage;
    private final StatusSignal<Double> m_signalDriveSupplyVoltage;
    private final StatusSignal<Double> m_signalDriveStatorCurrent;
    private final StatusSignal<Double> m_signalDriveSupplyCurrent;

    private final StatusSignal<Double> m_signalTurnPosition;
    private final StatusSignal<Double> m_signalTurnVelocity;
    private final StatusSignal<Double> m_signalTurnAcceleration;
    private final StatusSignal<Double> m_signalTurnTemp;
    private final StatusSignal<Double> m_signalTurnAppliedVoltage;
    private final StatusSignal<Double> m_signalTurnSupplyVoltage;
    private final StatusSignal<Double> m_signalTurnStatorCurrent;
    private final StatusSignal<Double> m_signalTurnSupplyCurrent;

    public SwerveModuleIOTalonFx(int drivePort, int turnPort, boolean useFOC) {
        m_drive = new TalonFX(drivePort);
        m_turn = new TalonFX(turnPort);

        m_useFOC = useFOC;

        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.kDriveStatorCurrentLimit;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kDriveSupplyCurrentLimit;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfigs.Slot0.kP = SwerveModuleConstants.kDrivePID.kp();
        driveConfigs.Slot0.kI = SwerveModuleConstants.kDrivePID.ki();
        driveConfigs.Slot0.kD = SwerveModuleConstants.kDrivePID.kd();
        m_drive.getConfigurator().apply(driveConfigs);

        TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
        turnConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kTurnSupplyCurrentLimit;
        turnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfigs.Slot0.kP = SwerveModuleConstants.kTurnPID.kp();
        turnConfigs.Slot0.kI = SwerveModuleConstants.kTurnPID.ki();
        turnConfigs.Slot0.kD = SwerveModuleConstants.kTurnPID.kd();
        m_turn.getConfigurator().apply(turnConfigs);

        m_driveVelocity = new VelocityVoltage(0);
        m_driveVelocity.EnableFOC = m_useFOC;

        m_turnPosition = new PositionVoltage(0);
        m_turnPosition.EnableFOC = m_useFOC;

        m_driveVoltage = new VoltageOut(0, m_useFOC, false, false, false);
        m_turnVoltage = new VoltageOut(0, m_useFOC, false, false, false);

        m_signalDrivePosition = m_drive.getPosition();
        m_signalDriveVelocity = m_drive.getVelocity();
        m_signalDriveAcceleration = m_drive.getAcceleration();
        m_signalDriveTemp = m_drive.getDeviceTemp();
        m_signalDriveAppliedVoltage = m_drive.getMotorVoltage();
        m_signalDriveSupplyVoltage = m_drive.getSupplyVoltage();
        m_signalDriveStatorCurrent = m_drive.getStatorCurrent();
        m_signalDriveSupplyCurrent = m_drive.getSupplyCurrent();

        m_signalTurnPosition = m_turn.getPosition();
        m_signalTurnVelocity = m_turn.getVelocity();
        m_signalTurnAcceleration = m_turn.getAcceleration();
        m_signalTurnTemp = m_turn.getDeviceTemp();
        m_signalTurnAppliedVoltage = m_turn.getMotorVoltage();
        m_signalTurnSupplyVoltage = m_turn.getSupplyVoltage();
        m_signalTurnStatorCurrent = m_turn.getStatorCurrent();
        m_signalTurnSupplyCurrent = m_turn.getSupplyCurrent();

        m_drive.setPosition(0);
        m_turn.setPosition(0);

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveConstants.kOdometryFrequencyHz,
            m_signalDrivePosition,
            m_signalDriveVelocity,
            m_signalTurnPosition,
            m_signalTurnVelocity
        );
    }

    @Override
    public void setDriveVoltage(double volts) {
        m_drive.setControl(m_driveVoltage.withOutput(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        m_turn.setControl(m_turnVoltage.withOutput(volts));
    }

    @Override
    public void setDriveSpeed(double speedRadPerSec) {
        m_drive.setControl(m_driveVelocity.withVelocity(Conversions.radiansToRotations(speedRadPerSec)));
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        m_turn.setControl(m_turnPosition.withPosition(position.getRotations()));
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            m_signalDrivePosition,
            m_signalDriveVelocity,
            m_signalDriveAcceleration,
            m_signalDriveTemp,
            m_signalDriveAppliedVoltage,
            m_signalDriveSupplyVoltage,
            m_signalDriveStatorCurrent,
            m_signalDriveSupplyCurrent,
            m_signalTurnPosition,
            m_signalTurnVelocity,
            m_signalTurnAcceleration,
            m_signalTurnTemp,
            m_signalTurnAppliedVoltage,
            m_signalTurnSupplyVoltage,
            m_signalTurnStatorCurrent,
            m_signalTurnSupplyCurrent
        );

        inputs.drivePositionRad = Conversions.driveMotorRotationsToDriveWheelRadians(
            m_signalDrivePosition.getValueAsDouble()
        );
        inputs.driveVelocityRadPerSec = Conversions.driveMotorRotationsToDriveWheelRadians(
            m_signalDriveVelocity.getValueAsDouble()
        );
        inputs.driveAccelerationRadPerSecPerSec = Conversions.driveMotorRotationsToDriveWheelRadians(
            m_signalDriveAcceleration.getValueAsDouble()
        );
        inputs.driveTempCelsius = m_signalDriveTemp.getValueAsDouble();
        inputs.driveAppliedVolts = m_signalDriveAppliedVoltage.getValueAsDouble();
        inputs.driveSupplyVoltage = m_signalDriveSupplyVoltage.getValueAsDouble();
        inputs.driveStatorCurrentAmps = m_signalDriveStatorCurrent.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = m_signalDriveSupplyCurrent.getValueAsDouble();

        inputs.turnPositionRad = Conversions.turnMotorRotationsToDriveWheelRadians(
            m_signalTurnPosition.getValueAsDouble()
        );
        inputs.turnVelocityRadPerSec = Conversions.turnMotorRotationsToDriveWheelRadians(
            m_signalTurnVelocity.getValueAsDouble()
        );
        inputs.turnAccelerationRadPerSecPerSec = Conversions.turnMotorRotationsToDriveWheelRadians(
            m_signalTurnAcceleration.getValueAsDouble()
        );
        inputs.turnTempCelsius = m_signalTurnTemp.getValueAsDouble();
        inputs.turnAppliedVolts = m_signalTurnAppliedVoltage.getValueAsDouble();
        inputs.turnSupplyVoltage = m_signalTurnSupplyVoltage.getValueAsDouble();
        inputs.turnStatorCurrentAmps = m_signalTurnStatorCurrent.getValueAsDouble();
        inputs.turnSupplyCurrentAmps = m_signalTurnSupplyCurrent.getValueAsDouble();
    }

    @Override
    public void setDriveBrake(boolean enabled) {
        m_drive.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setTurnBrake(boolean enabled) {
        m_turn.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {}
}
