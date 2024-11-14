package frc.robot.subsystems.swerve.swervemodule;

import org.littletonrobotics.junction.Logger;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedpidcontroller.LoggedPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIO.SwerveModuleOdometryInputs;
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

    // In motor space, not module space
    private double m_driveDesiredSpeedRadPerSec;
    private Rotation2d m_turnDesiredAngle;

    private LoggedPIDController m_drivePID;
    private PIDController m_turnPID;

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
            m_turnPID = SwerveModuleConstants.kTurnPID.getPIDController();

            m_driveDesiredSpeedRadPerSec = 0.0;
            m_turnDesiredAngle = Rotation2d.fromDegrees(0);
        }
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? SwerveModuleState.optimize(desiredState, getTurnAngle())
            : desiredState;

        m_driveDesiredSpeedRadPerSec = Conversions.driveWheelMetersToDriveMotorRadians(
            optimizedState.speedMetersPerSecond
        );

        m_turnDesiredAngle = Conversions.driveWheelAngleRotation2dToTurnMotorRotation2d(optimizedState.angle);

        m_desiredState = optimizedState;
        return optimizedState;
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState) {
        return setDesiredState(desiredState, true);
    }

    public Rotation2d getTurnAngle() {
        return Rotation2d.fromRadians(m_inputs.turnPositionRad);
    }

    public SwerveModuleOdometryInputs updateOdometryInputs() {
        return m_io.updateOdometryInputs();
    }

    public SwerveModuleState getState() {
        return m_currentState;
    }

    public SwerveModulePosition getPosition() {
        return m_currentPosition;
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(m_logPath, m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setDriveVoltage(0);
            m_io.setTurnVoltage(0);
        }
        else {
            // Update PID controllers if use them on rio
            if (m_useMotorPID) {
                m_io.setDriveSpeed(m_driveDesiredSpeedRadPerSec);

                m_io.setTurnPosition(m_turnDesiredAngle);
            }
            else {
                m_io.setDriveVoltage(
                    m_drivePID.calculate(m_inputs.driveVelocityRadPerSec, m_driveDesiredSpeedRadPerSec)
                );
                m_io.setTurnVoltage(m_turnPID.calculate(m_inputs.turnPositionRad, m_turnDesiredAngle.getRadians()));
            }
        }

        m_currentState.angle = Conversions.turnMotorRadiansToDriveWheelRotation2d(m_inputs.turnPositionRad);
        m_currentState.speedMetersPerSecond = Conversions.driveMotorRadiansToDriveWheelMeters(
            m_inputs.driveVelocityRadPerSec
        );

        m_currentPosition.angle = m_currentState.angle;
        m_currentPosition.distanceMeters = Conversions.driveMotorRadiansToDriveWheelMeters(m_inputs.drivePositionRad);

        Logger.recordOutput(m_logPath.concat("/desiredState"), m_desiredState);
        Logger.recordOutput(m_logPath.concat("/currentState"), m_currentState);
    }
}
