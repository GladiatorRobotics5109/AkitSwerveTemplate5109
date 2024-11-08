package frc.robot;

public final class Constants {
    public static enum Mode {
        kReal,
        kSim,
        kReplay
    }

    public static final Mode kCurrentMode = Mode.kSim;
}
