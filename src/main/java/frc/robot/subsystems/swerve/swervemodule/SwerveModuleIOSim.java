package frc.robot.subsystems.swerve.swervemodule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final DCMotorSim m_drive;
    private final DCMotorSim m_turn;

    private double m_driveAppliedVolts;
    private double m_turnAppliedVolts;

    public SwerveModuleIOSim() {
        m_drive = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                SwerveModuleConstants.kDriveFeedforward.kv(),
                SwerveModuleConstants.kDriveFeedforward.ka()
            ),
            DCMotor.getKrakenX60(1)
        );

        m_turn = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                SwerveModuleConstants.kTurnFeedforward.kv(),
                SwerveModuleConstants.kTurnFeedforward.ka()
            ),
            DCMotor.getKrakenX60(1)
        );
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        m_drive.update(Constants.kLoopPeriodSecs);
        m_turn.update(Constants.kLoopPeriodSecs);

        inputs.drivePositionRad = m_drive.getAngularPositionRad();
        inputs.driveVelocityRadPerSec = m_drive.getAngularVelocityRadPerSec();
        inputs.driveAccelerationRadPerSecPerSec = 0.0;
        inputs.driveTempCelsius = 0.0;
        inputs.driveAppliedVolts = m_driveAppliedVolts;
        inputs.driveSupplyVoltage = 12.0;
        inputs.driveStatorCurrentAmps = Math.abs(m_drive.getCurrentDrawAmps());
        inputs.driveSupplyCurrentAmps = m_drive.getCurrentDrawAmps() * m_driveAppliedVolts / 12;

        inputs.turnPositionRad = m_turn.getAngularPositionRad();
        inputs.turnVelocityRadPerSec = m_turn.getAngularVelocityRadPerSec();
        inputs.turnAccelerationRadPerSecPerSec = 0.0;
        inputs.turnTempCelsius = 0.0;
        inputs.turnAppliedVolts = m_turnAppliedVolts;
        inputs.turnSupplyVoltage = 12.0;
        inputs.turnStatorCurrentAmps = Math.abs(m_turn.getCurrentDrawAmps());
        inputs.turnSupplyCurrentAmps = m_turn.getCurrentDrawAmps() * m_turnAppliedVolts / 12;
    }

    @Override
    public void setDriveVoltage(double volts) {
        m_driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        m_drive.setInputVoltage(m_driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        m_turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        m_turn.setInputVoltage(m_turnAppliedVolts);
    }

    @Override
    public void setDriveWheelSpeed(double speedRadPerSec) {}

    @Override
    public void setTurnPosition(Rotation2d position) {}

    @Override
    public void periodic() {}
}
