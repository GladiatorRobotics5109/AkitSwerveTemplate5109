package frc.robot.util;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.UtilBase;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public final class Util extends UtilBase {
    public static final Alliance getAlliance() {
        return Util.getAlliance(Constants.kDefaultAlliance);
    }
}
