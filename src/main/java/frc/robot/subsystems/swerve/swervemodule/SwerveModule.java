package frc.robot.subsystems.swerve.swervemodule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIO.SwerveModuleOdometryInputs;
import frc.robot.util.Conversions;

public class SwerveModule {
    private final int m_id;

    private final SwerveModuleIO m_io;
    private final SwerveModuleIOInputsAutoLogged m_inputs;

    private final boolean m_useMotorPID;

    private SwerveModuleState m_desiredState;
    private SwerveModuleState m_currentState;

    public SwerveModule(int id, SwerveModuleIO io, boolean useMotorPID) {
        m_id = id;

        m_io = io;
        m_inputs = new SwerveModuleIOInputsAutoLogged();

        m_useMotorPID = useMotorPID;
    }

    public SwerveModuleState setDesiredState(SwerveModuleState desiredState, boolean optimize) {
        SwerveModuleState optimizedState = optimize ? SwerveModuleState.optimize(desiredState, getTurnAngle())
            : desiredState;

        if (m_useMotorPID) {
            m_io.setDriveSpeed(
                Units.RadiansPerSecond.of(
                    Conversions.wheelMetersToDriveMotorRadians(optimizedState.speedMetersPerSecond)
                )
            );

            m_io.setTurnPosition(Conversions.wheelAngleRotation2dToTurnMotorRotation2d(optimizedState.angle));
        }

        return optimizedState;
    }

    public Rotation2d getTurnAngle() {
        return Rotation2d.fromRadians(m_inputs.turnPositionRad);
    }

    public SwerveModuleOdometryInputs updateOdometryInputs() {
        return m_io.updateOdometryInputs();
    }

    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(String.format("Subsystems/Swerve/Module%i", m_id), m_inputs);

        if (DriverStation.isDisabled()) {
            m_io.setDriveVoltage(Units.Volts.of(0));
            m_io.setTurnVoltage(Units.Volts.of(0));
        }
    }
}
