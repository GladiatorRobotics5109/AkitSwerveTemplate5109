package frc.robot.subsystems.swerve;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants;
import com.github.gladiatorrobotics5109.gladiatorroboticslib.constants.swerveModuleConstants.SwerveDriveSpecialtiesConstants.MK4Constants.MK4GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public final class SwerveConstants {
    public static final class SwerveModuleConstants {
        /* Use PID controller on motor controllers */
        public static final boolean kUseMotorPID = true;
        /* Use FOC on TalonFX */
        public static final boolean kUseFOC = false;

        public static final MK4GearRatio kDriveGearRatio = MK4GearRatio.L1;
        public static final double kTurnGearRatio = MK4Constants.kTurnGearRatio;

        public static final Measure<Distance> kWheelRadius = Units.Meters.of(0.0472659347214289);
        
        // TODO: get correct ports
        public static final int kFrontLeftDrivePort = 0;
        public static final int kFrontLeftTurnPort = 0;
        public static final int kFrontRightDrivePort = 0;
        public static final int kFrontRightTurnPort = 0;
        public static final int kBackLeftDrivePort = 0;
        public static final int kBackLeftTurnPort = 0;
        public static final int kBackRightDrivePort = 0;
        public static final int kBackRightTurnPort = 0;

        public static final Translation2d kFrontLeftPos = new Translation2d();
        public static final Translation2d kFrontRightPos = new Translation2d();
        public static final Translation2d kBackLeftPos = new Translation2d();
        public static final Translation2d kBackRightPos = new Translation2d();
    }

    public static final double kOdometryFrequencyHz = 50;
    public static final Pose2d kStartingPose = new Pose2d();
}
