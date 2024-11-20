package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveDriveConfiguration;

public final class SwerveControllerFactory {
    private SwerveControllerFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static final class SwerveTeleopCommand extends Command {
        private final SwerveSubsystem m_swerve;
        private final SwerveDriveConfiguration m_config;
        private final DoubleSupplier m_x, m_y, m_rot, m_superSpeed;

        public SwerveTeleopCommand(
            SwerveSubsystem swerve,
            SwerveDriveConfiguration config,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rot,
            DoubleSupplier superSpeed
        ) {
            m_swerve = swerve;
            addRequirements(m_swerve);
            m_config = config;
            m_x = x;
            m_y = y;
            m_rot = rot;
            m_superSpeed = superSpeed;

            // Configure button bindings
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            double driveSpeedMetersPerSec = m_config.evalDriveSpeed(m_superSpeed.getAsDouble()).in(
                Units.MetersPerSecond
            );
            double vx = driveSpeedMetersPerSec * m_x.getAsDouble();
            double vy = driveSpeedMetersPerSec * m_y.getAsDouble();

            double rotationSpeedRadPerSec = m_config.evalRotationSpeed(m_superSpeed.getAsDouble()).in(
                Units.RadiansPerSecond
            );
            double vrot = rotationSpeedRadPerSec * m_rot.getAsDouble();

            m_swerve.drive(vx, vy, vrot, m_config.fieldRelative());
            // m_swerve.drive(0, 1.5, 0, m_config.fieldRelative());
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            m_swerve.drive(0, 0, 0, m_config.fieldRelative());
        }
    }

    public static Command makeTeleop(SwerveSubsystem swerve, CommandXboxController controller) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX,
            controller::getRightTriggerAxis
        );
    }

    public static Command makeTeleop(
        SwerveSubsystem swerve,
        DoubleSupplier translateX,
        DoubleSupplier translateY,
        DoubleSupplier rot,
        DoubleSupplier superSpeed
    ) {
        return new SwerveTeleopCommand(
            swerve,
            SwerveConstants.kTeleopConfig,
            translateX,
            translateY,
            rot,
            superSpeed
        );
    }
}
