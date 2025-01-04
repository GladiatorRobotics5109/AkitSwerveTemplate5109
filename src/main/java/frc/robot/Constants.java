package frc.robot;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.Mode;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
    public static final Mode kCurrentMode = Mode.REAL;

    public static final Alliance kDefaultAlliance = Alliance.Blue;

    public static final double kLoopPeriodSecs = Robot.defaultPeriodSecs;

    public static final int kPDPPort = 0;

    public static final double kJoystickDeadzone = 0.15;
}
