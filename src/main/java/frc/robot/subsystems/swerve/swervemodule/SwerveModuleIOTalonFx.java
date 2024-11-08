package frc.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.subsystems.swerve.SwerveConstants;
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

    private SwerveModuleOdometryInputs m_latestOdometryInputs;

    public SwerveModuleIOTalonFx(int drivePort, int turnPort, boolean useFOC) {
        m_drive = new TalonFX(drivePort);
        m_turn = new TalonFX(turnPort);

        m_useFOC = useFOC;

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

        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kOdometryFrequencyHz, m_signalDrivePosition, m_signalDriveVelocity, m_signalTurnPosition, m_signalTurnVelocity);
    }

    @Override
    public void setDriveVoltage(Measure<Voltage> volts) {
        m_drive.setControl(m_driveVoltage.withOutput(volts.in(Units.Volts)));
    }

    @Override
    public void setTurnVoltage(Measure<Voltage> volts) {
        m_turn.setControl(m_turnVoltage.withOutput(volts.in(Units.Volts)));
    }

    @Override
    public void setDriveSpeed(Measure<Velocity<Angle>> speed) {
        m_drive.setControl(m_driveVelocity.withVelocity(speed.in(Units.RotationsPerSecond)));
    }

    @Override
    public void setTurnPosition(Rotation2d position) {
        m_turn.setControl(m_turnPosition.withPosition(position.getRotations()));
    }
    
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRad = m_latestOdometryInputs.drivePositionRad();
        inputs.driveVelocityRadPerSec = m_latestOdometryInputs.driveVelocityRadPerSec();
        inputs.driveAccelerationRadPerSecPerSec = m_latestOdometryInputs.driveAccelerationRadPerSecPerSec();
        inputs.driveTempCelsius = m_signalDriveTemp.getValueAsDouble();
        inputs.driveAppliedVolts = m_signalDriveAppliedVoltage.getValueAsDouble();
        inputs.driveSupplyVoltage = m_signalDriveSupplyVoltage.getValueAsDouble();
        inputs.driveStatorCurrentAmps = m_signalDriveStatorCurrent.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = m_signalDriveSupplyCurrent.getValueAsDouble();

        inputs.turnPositionRad = m_latestOdometryInputs.turnPositionRad();
        inputs.turnVelocityRadPerSec = m_latestOdometryInputs.turnVelocityRadPerSec();
        inputs.turnAccelerationRadPerSecPerSec = Conversions.rotationsToRadians(m_signalTurnAcceleration.getValueAsDouble());
        inputs.turnTempCelsius = m_signalTurnTemp.getValueAsDouble();
        inputs.turnAppliedVolts = m_signalTurnAppliedVoltage.getValueAsDouble();
        inputs.turnSupplyVoltage = m_signalTurnSupplyVoltage.getValueAsDouble();
        inputs.turnStatorCurrentAmps = m_signalTurnStatorCurrent.getValueAsDouble();
        inputs.turnSupplyCurrentAmps = m_signalTurnSupplyCurrent.getValueAsDouble();
    }

    @Override
    public SwerveModuleOdometryInputs updateOdometryInputs() {
        BaseStatusSignal.refreshAll(m_signalDrivePosition, m_signalDriveVelocity, m_signalDriveAcceleration, m_signalTurnPosition, m_signalTurnVelocity);

        m_latestOdometryInputs = new SwerveModuleOdometryInputs(Conversions.rotationsToRadians(m_signalDrivePosition.getValueAsDouble()), Conversions.rotationsToRadians(m_signalDriveVelocity.getValueAsDouble()), Conversions.rotationsToRadians(m_signalDriveAcceleration.getValueAsDouble()), Conversions.rotationsToRadians(m_signalTurnPosition.getValueAsDouble()), Conversions.rotationsToRadians(m_signalTurnVelocity.getValueAsDouble()));

        return m_latestOdometryInputs;
    }

    public void periodic() {
        // Refresh all non odometry signals
        BaseStatusSignal.refreshAll(
            m_signalDriveTemp,
            m_signalDriveAppliedVoltage,
            m_signalDriveSupplyVoltage,
            m_signalDriveStatorCurrent,
            m_signalDriveSupplyCurrent,
            m_signalTurnAcceleration,
            m_signalTurnTemp,
            m_signalTurnTemp,
            m_signalTurnAppliedVoltage,
            m_signalTurnSupplyVoltage,
            m_signalTurnStatorCurrent,
            m_signalTurnSupplyCurrent
        );
    }
}
