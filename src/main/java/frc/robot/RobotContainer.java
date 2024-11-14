// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.Mode;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveControllerFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
    public static PowerDistribution powerDistribution;
    private final SwerveSubsystem m_swerve;

    private final CommandXboxController m_driverController;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();
        if (Constants.kCurrentMode == Mode.REAL) {
            powerDistribution = new PowerDistribution(Constants.kPDPPort, ModuleType.kAutomatic);
        }

        m_driverController = new CommandXboxController(0);

        configureBindings();
    }

    private void configureBindings() {
        m_swerve.setController(SwerveControllerFactory.makeTeleop(m_swerve, m_driverController));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
