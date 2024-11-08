package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveOdometryThread extends Thread {
    private final Supplier<SwerveModulePosition[]> m_modulePositionSuppliers;
    private SwerveModulePosition[] m_modulePositions;
    private final Supplier<Rotation2d> m_gyroAngle;

    private final SwerveDrivePoseEstimator m_poseEstimator;
    private Pose2d m_latestPose;
    private final Lock m_poseLock;

    public SwerveOdometryThread(
        Supplier<SwerveModulePosition[]> modulePositionSuppliers,
        Supplier<Rotation2d> gyroAngle,
        SwerveDriveKinematics kinematics
    ) {
        m_modulePositionSuppliers = modulePositionSuppliers;
        m_modulePositions = new SwerveModulePosition[4];
        m_gyroAngle = gyroAngle;

        m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            m_gyroAngle.get(),
            m_modulePositions,
            SwerveConstants.kStartingPose
        );

        m_poseLock = new ReentrantLock();
    }

    public void run() {
        while (true) {
            try {
                m_poseLock.lock();

                m_modulePositions = m_modulePositionSuppliers.get();

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
