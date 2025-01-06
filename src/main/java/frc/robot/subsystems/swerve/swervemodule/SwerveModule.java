package frc.robot.subsystems.swerve.swervemodule;

import org.littletonrobotics.junction.Logger;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedpidcontroller.LoggedPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModule {
    private final int m_id;
    private final String m_logPath;

    private final SwerveModuleIO m_io;
    private final SwerveModuleIOInputsAutoLogged m_inputs;

    private final boolean m_useMotorPID;

    private SwerveModuleState m_desiredState;
    private SwerveModuleState m_currentState;
    private SwerveModulePosition m_currentPosition;

    private LoggedPIDController m_drivePID;
    private LoggedPIDController m_turnPID;

    private SimpleMotorFeedforward m_driveFeedforward;
    private SimpleMotorFeedforward m_turnFeedforward;

    public SwerveModule(int id, SwerveModuleIO io, boolean useMotorPID) {
        m_id = id;
        m_logPath = SwerveConstants.kLogPath + String.format("/SwerveModule%d", m_id);

        m_io = io;
        m_inputs = new SwerveModuleIOInputsAutoLogged();

        m_useMotorPID = useMotorPID;

        m_desiredState = new SwerveModuleState();
        m_currentState = new SwerveModuleState();
        m_currentPosition = new SwerveModulePosition();

        if (!m_useMotorPID) {
            m_drivePID = SwerveModuleConstants.kDrivePID.getLoggedPIDController(m_logPath + "/drivePID");
            m_turnPID = SwerveModuleConstants.kTurnPID.getLoggedPIDController(m_logPath + "/turnPID");
            m_driveFeedforward = SwerveModuleConstants.kDriveFeedforward.get();
            m_turnFeedforward = SwerveModuleConstants.kTurnFeedforward.get();
        }
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        SwerveModuleState optimizedState = optimize
            ? SwerveModuleState.optimize(desiredState, getTurnAngle())
            : desiredState;

        m_desiredState = optimizedState;
        return optimizedState;
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
        return setDesiredState(desiredState, true);
    }

    public void setDriveVoltage(double volts) {
        m_io.setDriveVoltage(volts);
    }

    public void setTurnVoltage(double volts) {
        m_io.setTurnVoltage(volts);
    }

    public Rotation2d getTurnAngle() {
        // return m_desiredState.angle;
        return m_currentState.angle;
    }

    public SwerveModuleState getState() {
        return m_currentState;
    }

    public SwerveModulePosition getPosition() {
        return m_currentPosition;
    }

    public void periodic() {
        m_io.periodic();
        m_io.updateInputs(m_inputs);
        Logger.processInputs(m_logPath, m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setDriveVoltage(0);
            m_io.setTurnVoltage(0);
        }
        else {
            if (m_useMotorPID) {
                m_io.setDriveWheelSpeed(
                    Conversions.driveWheelMetersToWheelRadians(m_desiredState.speedMetersPerSecond)
                );

                m_io.setTurnPosition(m_desiredState.angle);
            }
            else {
                m_io.setDriveVoltage(
                    m_driveFeedforward.calculate(
                        Conversions.driveWheelMetersToWheelRadians(m_desiredState.speedMetersPerSecond)
                    )
                        + m_drivePID.calculate(
                            Conversions.driveWheelMetersToWheelRadians(m_currentState.speedMetersPerSecond),
                            Conversions.driveWheelMetersToWheelRadians(m_desiredState.speedMetersPerSecond)
                        )
                );
                m_io.setTurnVoltage(
                    m_turnFeedforward.calculate(
                        m_desiredState.angle.getRadians()
                    )
                        + m_turnPID.calculate(m_currentState.angle.getRadians(), m_desiredState.angle.getRadians())
                );
            }
        }

        m_currentState.angle = Rotation2d.fromRadians(m_inputs.turnPositionRad);
        m_currentState.speedMetersPerSecond = Conversions.driveWheelRadiansToWheelMeters(
            m_inputs.driveVelocityRadPerSec
        );

        m_currentPosition.angle = m_currentState.angle;
        m_currentPosition.distanceMeters = Conversions.driveWheelRadiansToWheelMeters(m_inputs.drivePositionRad);

        Logger.recordOutput(m_logPath.concat("/desiredState"), m_desiredState);
        Logger.recordOutput(m_logPath.concat("/currentState"), m_currentState);
    }
}
