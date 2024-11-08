package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.math.ConversionsBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Conversions extends ConversionsBase {
    public static final double wheelMetersToDriveMotorRadians(double m) {
        return ConversionsBase.wheelMetersToDriveMotorRadians(
            m,
            SwerveConstants.SwerveModuleConstants.kWheelRadius.in(Units.Meters),
            SwerveConstants.SwerveModuleConstants.kDriveGearRatio
        );
    }

    public static final double driveMotorRotationsToDriveWheelRadians(double rot) {
        return ConversionsBase.driveMotorRotationsToDriveWheelRadians(rot, SwerveConstants.SwerveModuleConstants.kDriveGearRatio);
    }

    public static final Rotation2d wheelAngleRotation2dToTurnMotorRotation2d(Rotation2d angle) {
        return ConversionsBase.wheelAngleRotation2dToTurnMotorRotation2d(
            angle,
            SwerveConstants.SwerveModuleConstants.kTurnGearRatio
        );
    }
}
