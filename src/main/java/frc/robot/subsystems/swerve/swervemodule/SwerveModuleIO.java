package frc.robot.subsystems.swerve.swervemodule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.util.Conversions;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAccelerationRadPerSecPerSec = 0.0;
        public double driveTempCelsius = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveSupplyVoltage = 0.0;
        public double driveStatorCurrentAmps = 0.0;
        public double driveSupplyCurrentAmps = 0.0;

        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAccelerationRadPerSecPerSec = 0.0;
        public double turnTempCelsius = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnSupplyVoltage = 0.0;
        public double turnStatorCurrentAmps = 0.0;
        public double turnSupplyCurrentAmps = 0.0;

        // public double[] odometryTimestamps = new double[] {};
        // public double[] odometryDrivePositionsRad = new double[] {};
        // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    // This can probably only hold drive position and turn position
    public static final record SwerveModuleOdometryInputs(
        double drivePositionRad,
        double driveVelocityRadPerSec,
        double driveAccelerationRadPerSecPerSec,
        double turnPositionRad,
        double turnVelocityRadPerSec
    ) {
        public final static SwerveModulePosition toModulePosition(SwerveModuleOdometryInputs inputs) {
            return new SwerveModulePosition(Conversions.driveMotorRotationsToDriveWheelRadians(inputs.drivePositionRad), Conversions.turnMotorRotationsToWheelRotation2d(inputs.turnPositionRad));
        }

        public final SwerveModulePosition toModulePosition() {
            return toModulePosition(this);
        }
    }

    default void setDriveVoltage(Measure<Voltage> volts) {}

    default void setTurnVoltage(Measure<Voltage> volts) {}

    default void setDriveSpeed(Measure<Velocity<Angle>> speed) {}

    default void setTurnPosition(Rotation2d position) {}

    default void updateInputs(SwerveModuleIOInputs inputs) {}

    default SwerveModuleOdometryInputs updateOdometryInputs() { return new SwerveModuleOdometryInputs(0, 0, 0, 0, 0); }
}
