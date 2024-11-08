package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyro;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIO;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIONavX;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.loggedgyro.LoggedGyroIOSim;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIO;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIOSim;
import frc.robot.subsystems.swerve.swervemodule.SwerveModuleIOTalonFx;

public class SwerveSubsystem {
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    private final LoggedGyro m_gyro;

    private final SwerveDriveKinematics m_kinematics;

    private final SwerveOdometryThread m_odometry;

    public SwerveSubsystem() {
        switch (Constants.kCurrentMode) {
            case REAL:
                m_moduleFL = new SwerveModule(0, new SwerveModuleIOTalonFx(SwerveConstants.SwerveModuleConstants.kFrontLeftDrivePort, SwerveConstants.SwerveModuleConstants.kFrontLeftTurnPort, SwerveConstants.SwerveModuleConstants.kUseFOC), SwerveConstants.SwerveModuleConstants.kUseMotorPID);
                m_moduleFR = new SwerveModule(1, new SwerveModuleIOTalonFx(SwerveConstants.SwerveModuleConstants.kFrontRightDrivePort, SwerveConstants.SwerveModuleConstants.kFrontRightTurnPort, SwerveConstants.SwerveModuleConstants.kUseFOC), SwerveConstants.SwerveModuleConstants.kUseMotorPID);
                m_moduleBL = new SwerveModule(2, new SwerveModuleIOTalonFx(SwerveConstants.SwerveModuleConstants.kBackLeftDrivePort, SwerveConstants.SwerveModuleConstants.kBackLeftTurnPort, SwerveConstants.SwerveModuleConstants.kUseFOC), SwerveConstants.SwerveModuleConstants.kUseMotorPID);
                m_moduleBR = new SwerveModule(3, new SwerveModuleIOTalonFx(SwerveConstants.SwerveModuleConstants.kBackRightDrivePort, SwerveConstants.SwerveModuleConstants.kBackRightTurnPort, SwerveConstants.SwerveModuleConstants.kUseFOC), SwerveConstants.SwerveModuleConstants.kUseMotorPID);

                m_gyro = new LoggedGyro("Subsystems/Swerve/Gyro", new LoggedGyroIONavX());

                break;
            case SIM:
                m_moduleFL = new SwerveModule(0, new SwerveModuleIOSim(), false);
                m_moduleFR = new SwerveModule(1, new SwerveModuleIOSim(), false);
                m_moduleBL = new SwerveModule(2, new SwerveModuleIOSim(), false);
                m_moduleBR = new SwerveModule(3, new SwerveModuleIOSim(), false);

                m_gyro = new LoggedGyro("Subsystems/Swerve/Gyro", new LoggedGyroIOSim());

                break;
            default:
                m_moduleFL = new SwerveModule(0, new SwerveModuleIO() {}, false);
                m_moduleFR = new SwerveModule(1, new SwerveModuleIO() {}, false);
                m_moduleBL = new SwerveModule(2, new SwerveModuleIO() {}, false);
                m_moduleBR = new SwerveModule(3, new SwerveModuleIO() {}, false);

                m_gyro = new LoggedGyro("Subsystems/Swerve/Gyro", new LoggedGyroIO() {});

                break;
        }

        m_kinematics = new SwerveDriveKinematics(
            SwerveConstants.SwerveModuleConstants.kFrontLeftPos,
            SwerveConstants.SwerveModuleConstants.kFrontRightPos,
            SwerveConstants.SwerveModuleConstants.kBackLeftPos,
            SwerveConstants.SwerveModuleConstants.kBackRightPos
        );
        
        // TODO: make the gyro update on odometry thread
        m_odometry = new SwerveOdometryThread(() -> {
            return new SwerveModulePosition[] {
                m_moduleFL.updateOdometryInputs().toModulePosition(),
                m_moduleFR.updateOdometryInputs().toModulePosition(),
                m_moduleBL.updateOdometryInputs().toModulePosition(),
                m_moduleBR.updateOdometryInputs().toModulePosition(),
            };
        }, m_gyro::getYaw, m_kinematics);

        m_odometry.start();
    }
}
