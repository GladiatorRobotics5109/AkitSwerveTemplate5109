package frc.robot.subsystems.swerve;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;

public class SwerveOdometryThread extends Thread {
    private SwerveModule[] m_modules;
    private SwerveModulePosition[] m_modulePositions;
    private final Supplier<Rotation2d> m_gyroAngle;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private Pose2d m_latestPose;
    private final Lock m_poseLock;
    private StatusSignal<?>[] m_statusSignals;

    public SwerveOdometryThread(
        SwerveModule[] modules,
        Supplier<Rotation2d> gyroAngle,
        SwerveDriveKinematics kinematics
    ) {
        m_modules = modules;
        m_modulePositions = new SwerveModulePosition[4];
        m_statusSignals = new StatusSignal[0];
        updateModulePositions();
        m_gyroAngle = gyroAngle;

        m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            m_gyroAngle.get(),
            m_modulePositions,
            SwerveConstants.kStartingPose
        );
        m_latestPose = SwerveConstants.kStartingPose;

        m_poseLock = new ReentrantLock();
    }

    public void registerOdometrySignals(StatusSignal<?>... signals) {
        int oldLen = m_statusSignals.length;
        m_statusSignals = Arrays.copyOf(m_statusSignals, oldLen + signals.length);
        for (int i = oldLen; i < m_statusSignals.length; i++) {
            m_statusSignals[i] = signals[i];
        }
    }

    public void updateModulePositions() {
        if (m_statusSignals.length != 0)
            BaseStatusSignal.refreshAll(m_statusSignals);
        for (int i = 0; i < m_modules.length; i++) {
            m_modulePositions[i] = m_modules[i].updateOdometryInputs().toModulePosition();
        }
    }

    public void run() {
        while (true) {
            try {
                m_poseLock.lock();

                updateModulePositions();

                m_poseEstimator.updateWithTime(Logger.getRealTimestamp(), m_gyroAngle.get(), m_modulePositions);
                m_latestPose = m_poseEstimator.getEstimatedPosition();

                Thread.sleep((long)(1000 / SwerveConstants.kOdometryFrequencyHz));
            }
            catch (InterruptedException e) {
                DriverStation.reportWarning("Odometry thread has been interrupted!", e.getStackTrace());
            }
            finally {
                m_poseLock.unlock();
            }
        }
    }

    public Pose2d getLatest() {
        try {
            m_poseLock.lock();
            // Should to copy here right?
            return new Pose2d(m_latestPose.getTranslation(), m_latestPose.getRotation());
        }
        finally {
            m_poseLock.unlock();
        }
    }
}
