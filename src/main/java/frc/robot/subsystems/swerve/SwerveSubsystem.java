package frc.robot.subsystems.swerve;


import frc.robot.Constants;
import frc.robot.subsystems.swerve.swervemodule.SwerveModule;

public class SwerveSubsystem {
    private final SwerveModule m_moduleFL;
    private final SwerveModule m_moduleFR;
    private final SwerveModule m_moduleBL;
    private final SwerveModule m_moduleBR;

    public SwerveSubsystem() {
        switch (Constants.kCurrentMode) {
            case kSim:

                break;
            case kReplay:
                break;
            case kReal:
                m_moduleFL = new SwerveModule();
                break;
        }

    }
}
