package frc.robot.subsystems.swerve;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants.MK4GearRatio;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.controller.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import frc.robot.util.Conversions;

public final class SwerveConstants {
    public static final class SwerveModuleConstants {
        /* Use PID controller on motor controllers */
        public static final boolean kUseMotorPID = true;
        /* Use FOC on TalonFX */
        public static final boolean kUseFOC = false;

        public static final MK4GearRatio kDriveGearRatio = MK4GearRatio.L1;
        public static final double kTurnGearRatio = MK4Constants.kTurnGearRatio;

        public static final double kWheelRadiusMeters = 0.0472659347214289;

        // TODO: get correct ports
        public static final int kFrontLeftDrivePort = 0;
        public static final int kFrontLeftTurnPort = 0;
        public static final int kFrontRightDrivePort = 0;
        public static final int kFrontRightTurnPort = 0;
        public static final int kBackLeftDrivePort = 0;
        public static final int kBackLeftTurnPort = 0;
        public static final int kBackRightDrivePort = 0;
        public static final int kBackRightTurnPort = 0;

        public static final Translation2d kModulePosFL = new Translation2d(0.290449, 0.290449);
        public static final Translation2d kModulePosFR = new Translation2d(0.290449, -0.290449);;
        public static final Translation2d kModulePosBL = new Translation2d(-0.290449, 0.290449);
        public static final Translation2d kModulePosBR = new Translation2d(-0.290449, -0.290449);

        public static final PIDConstants kDrivePID = new PIDConstants(
            Conversions.driveWheelRotationsToDriveMotorRadians(1 / 20),
            0,
            0
        );
        public static final PIDConstants kTurnPID = new PIDConstants(1, 0, 0);
    }

    public static final boolean kTeleopFieldRelative = true;

    public static final double kOdometryFrequencyHz = 50;
    public static final Pose2d kStartingPose = new Pose2d();
    public static final String kLogPath = "Subsystems/Swerve";

    public static final SwerveDriveConfiguration kTeleopConfig = new SwerveDriveConfiguration(
        Units.MetersPerSecond.of(2.5),
        Units.FeetPerSecond.of(12.9),
        Units.RotationsPerSecond.of(1),
        Units.RotationsPerSecond.of(1.5),
        kTeleopFieldRelative
    );

    // Provides a way to describe the configuration of the swerve subsystem (like drive speeds) with values that may
    // change throughout a match (like auto -> teleop)
    public static final record SwerveDriveConfiguration(Measure<Velocity<Distance>> defaultDriveSpeed, Measure<Velocity<Distance>> maxDriveSpeed, Measure<Velocity<Angle>> defaultRotationSpeed, Measure<Velocity<Angle>> maxRotationSpeed, boolean fieldRelative) {
        public Measure<Velocity<Distance>> evalDriveSpeed(double t) {
            // l(t) = (f / i) * t + i
            return maxDriveSpeed.divide(defaultDriveSpeed.in(maxDriveSpeed.unit())).times(t).plus(defaultDriveSpeed);
        }

        public Measure<Velocity<Angle>> evalRotationSpeed(double t) {
            // l(t) = (f / i) * t + i
            return maxRotationSpeed.divide(defaultRotationSpeed.in(maxRotationSpeed.unit())).times(t).plus(defaultRotationSpeed);
        }
    }
}
