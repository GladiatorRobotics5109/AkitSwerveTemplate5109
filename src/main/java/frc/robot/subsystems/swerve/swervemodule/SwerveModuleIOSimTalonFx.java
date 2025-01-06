package frc.robot.subsystems.swerve.swervemodule;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveModuleConstants;
import frc.robot.util.Conversions;

public class SwerveModuleIOSimTalonFx extends SwerveModuleIOTalonFx {
    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_turnSim;

    public SwerveModuleIOSimTalonFx(int drivePort, int turnPort, boolean useFOC) {
        super(drivePort, turnPort, useFOC);

        m_driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                SwerveModuleConstants.kDriveFeedforward.kv(),
                SwerveModuleConstants.kDriveFeedforward.ka()
            ),
            DCMotor.getKrakenX60(1)
        );

        m_turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                SwerveModuleConstants.kTurnFeedforward.kv(),
                SwerveModuleConstants.kTurnFeedforward.ka()
            ),
            DCMotor.getKrakenX60(1)
        );

        // m_driveSim = new DCMotorSim(
        // useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
        // SwerveModuleConstants.kDriveGearRatio.asDouble(),
        // // 1,
        // 0.025
        // );
        // m_turnSim = new DCMotorSim(
        // useFOC ? DCMotor.getKrakenX60Foc(1) : DCMotor.getKrakenX60(1),
        // SwerveModuleConstants.kTurnGearRatio,
        // // 1,
        // 0.004
        // );
    }

    @Override
    public void periodic() {
        // Update sim
        TalonFXSimState driveSimState = m_drive.getSimState();
        TalonFXSimState turnSimState = m_turn.getSimState();

        driveSimState.setSupplyVoltage(12);
        turnSimState.setSupplyVoltage(12);

        m_driveSim.setInputVoltage(driveSimState.getMotorVoltage());
        m_turnSim.setInputVoltage(turnSimState.getMotorVoltage());
        m_driveSim.update(Constants.kLoopPeriodSecs);
        m_turnSim.update(Constants.kLoopPeriodSecs);

        driveSimState.setRawRotorPosition(
            m_driveSim.getAngularPositionRotations() * SwerveModuleConstants.kDriveGearRatio.asDouble()
        );
        driveSimState.setRotorVelocity(
            Conversions.radiansToRotations(
                m_driveSim.getAngularVelocityRadPerSec() * SwerveModuleConstants.kDriveGearRatio.asDouble()
            )
        );

        turnSimState.setRawRotorPosition(
            m_turnSim.getAngularPositionRotations() * SwerveModuleConstants.kTurnGearRatio
        );
        turnSimState.setRotorVelocity(
            Conversions.radiansToRotations(
                m_turnSim.getAngularVelocityRadPerSec() * SwerveModuleConstants.kTurnGearRatio
            )
        );
    }
}
