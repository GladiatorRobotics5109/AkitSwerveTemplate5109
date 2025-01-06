package frc.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModuleIOTalonFx implements SwerveModuleIO {
    protected final TalonFX m_drive;
    protected final TalonFX m_turn;

    private final boolean m_useFOC;

    private final VoltageOut m_driveVoltage;
    private final VoltageOut m_turnVoltage;

    private final VelocityVoltage m_driveVelocity;
    private final PositionVoltage m_turnPosition;

    private final StatusSignal<Angle> m_signalDrivePosition;
    private final StatusSignal<AngularVelocity> m_signalDriveVelocity;
    private final StatusSignal<AngularAcceleration> m_signalDriveAcceleration;
    private final StatusSignal<Temperature> m_signalDriveTemp;
    private final StatusSignal<Voltage> m_signalDriveAppliedVoltage;
    private final StatusSignal<Voltage> m_signalDriveSupplyVoltage;
    private final StatusSignal<Current> m_signalDriveStatorCurrent;
    private final StatusSignal<Current> m_signalDriveSupplyCurrent;

    private final StatusSignal<Angle> m_signalTurnPosition;
    private final StatusSignal<AngularVelocity> m_signalTurnVelocity;
    private final StatusSignal<AngularAcceleration> m_signalTurnAcceleration;
    private final StatusSignal<Temperature> m_signalTurnTemp;
    private final StatusSignal<Voltage> m_signalTurnAppliedVoltage;
    private final StatusSignal<Voltage> m_signalTurnSupplyVoltage;
    private final StatusSignal<Current> m_signalTurnStatorCurrent;
    private final StatusSignal<Current> m_signalTurnSupplyCurrent;

    public SwerveModuleIOTalonFx(int drivePort, int turnPort, boolean useFOC) {
        m_drive = new TalonFX(drivePort);
        m_turn = new TalonFX(turnPort);

        m_useFOC = useFOC;

        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.kDriveStatorCurrentLimit;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kDriveSupplyCurrentLimit;
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // TODO: test new ratios
        driveConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kDriveGearRatio.asDouble();
        // Need to convert to rations here bc talonfx native units is rotations
        driveConfigs.Slot0.kP = Conversions.radiansToRotations(SwerveModuleConstants.kDrivePID.kp());
        driveConfigs.Slot0.kI = Conversions.radiansToRotations(SwerveModuleConstants.kDrivePID.ki());
        driveConfigs.Slot0.kD = Conversions.radiansToRotations(SwerveModuleConstants.kDrivePID.kd());
        driveConfigs.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        driveConfigs.Slot0.kS = Conversions.radiansToRotations(SwerveModuleConstants.kDriveFeedforward.ks());
        driveConfigs.Slot0.kV = Conversions.radiansToRotations(SwerveModuleConstants.kDriveFeedforward.kv());
        driveConfigs.Slot0.kA = Conversions.radiansToRotations(SwerveModuleConstants.kDriveFeedforward.ka());
        m_drive.getConfigurator().apply(driveConfigs);

        TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
        turnConfigs.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kTurnSupplyCurrentLimit;
        turnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants.kTurnGearRatio;
        turnConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        // Need to convert to rations here bc talonfx native units is rotations
        turnConfigs.Slot0.kP = Conversions.radiansToRotations(SwerveModuleConstants.kTurnPID.kp());
        turnConfigs.Slot0.kI = Conversions.radiansToRotations(SwerveModuleConstants.kTurnPID.ki());
        turnConfigs.Slot0.kD = Conversions.radiansToRotations(SwerveModuleConstants.kTurnPID.kd());
        turnConfigs.Slot0.kS = Conversions.radiansToRotations(SwerveModuleConstants.kTurnFeedforward.ks());
        turnConfigs.Slot0.kV = Conversions.radiansToRotations(SwerveModuleConstants.kTurnFeedforward.kv());
        turnConfigs.Slot0.kA = Conversions.radiansToRotations(SwerveModuleConstants.kTurnFeedforward.ka());
        m_turn.getConfigurator().apply(turnConfigs);

        m_driveVelocity = new VelocityVoltage(0);
        m_driveVelocity.EnableFOC = m_useFOC;

        m_turnPosition = new PositionVoltage(0);
        m_turnPosition.EnableFOC = m_useFOC;

        m_driveVoltage = new VoltageOut(0);
        m_driveVoltage.EnableFOC = m_useFOC;
        m_turnVoltage = new VoltageOut(0);
        m_turnVoltage.EnableFOC = m_useFOC;

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
            m_signalDriveAcceleration,
            m_signalTurnPosition,
            m_signalTurnVelocity,
            m_signalTurnAcceleration
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
    public void setDriveWheelSpeed(double wheelSpeedRadPerSec) {
        m_drive.setControl(m_driveVelocity.withVelocity(Conversions.radiansToRotations(wheelSpeedRadPerSec)));
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

        inputs.drivePositionRad = m_signalDrivePosition.getValue().in(Units.Radians);
        inputs.driveVelocityRadPerSec = m_signalDriveVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.driveAccelerationRadPerSecPerSec = m_signalDriveAcceleration.getValue().in(
            Units.RadiansPerSecondPerSecond
        );
        inputs.driveTempCelsius = m_signalDriveTemp.getValue().in(Units.Celsius);
        inputs.driveAppliedVolts = m_signalDriveAppliedVoltage.getValue().in(Units.Volts);
        inputs.driveSupplyVoltage = m_signalDriveSupplyVoltage.getValue().in(Units.Volts);
        inputs.driveStatorCurrentAmps = m_signalDriveStatorCurrent.getValue().in(Units.Amps);
        inputs.driveSupplyCurrentAmps = m_signalDriveSupplyCurrent.getValue().in(Units.Amps);

        inputs.turnPositionRad = m_signalTurnPosition.getValue().in(Units.Radians);
        inputs.turnVelocityRadPerSec = m_signalTurnVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.turnAccelerationRadPerSecPerSec = m_signalTurnAcceleration.getValue().in(
            Units.RadiansPerSecondPerSecond
        );
        inputs.turnTempCelsius = m_signalTurnTemp.getValue().in(Units.Celsius);
        inputs.turnAppliedVolts = m_signalTurnAppliedVoltage.getValue().in(Units.Volts);
        inputs.turnSupplyVoltage = m_signalTurnSupplyVoltage.getValue().in(Units.Volts);
        inputs.turnStatorCurrentAmps = m_signalTurnStatorCurrent.getValue().in(Units.Amps);
        inputs.turnSupplyCurrentAmps = m_signalTurnSupplyCurrent.getValue().in(Units.Amps);
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
