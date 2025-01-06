// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.github.gladiatorrobotics5109.gladiatorroboticslib.advantagekitutil.Mode;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveCommandFactory;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
    public static PowerDistribution powerDistribution;
    private final SwerveSubsystem m_swerve;

    private final CommandXboxController m_driverController;
    // private final CommandPS5Controller m_driverController;
    // private final CommandGenericHID m_keyboard;

    public RobotContainer() {
        m_swerve = new SwerveSubsystem();
        if (Constants.kCurrentMode == Mode.REAL) {
            powerDistribution = new PowerDistribution(Constants.kPDPPort, ModuleType.kCTRE);
        }

        m_driverController = new CommandXboxController(0);
        // m_driverController = new CommandPS5Controller(0);
        // m_keyboard = new CommandGenericHID(0);

        configureBindings();
    }

    private void configureBindings() {
        m_swerve.setDefaultCommand(SwerveCommandFactory.makeTeleop(m_swerve, m_driverController));
        // m_swerve.setDefaultCommand(
        // SwerveComSmandFactory.makeTeleop(
        // m_swerve,
        // // () -> 0,
        // // () -> 1,
        // () -> m_keyboard.getRawAxis(0),
        // () -> -m_keyboard.getRawAxis(1),
        // () -> m_keyboard.getRawAxis(2),
        // () -> 0
        // )
        // );
    }

    public Command getAutonomousCommand() {
        return null;
        // return Commands.sequence(
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).quasistatic(SysIdRoutine.Direction.kForward),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).quasistatic(SysIdRoutine.Direction.kReverse),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).dynamic(SysIdRoutine.Direction.kForward),
        // Commands.waitSeconds(2),
        // SwerveControllerFactory.makeSysIdDrive(m_swerve, 0).dynamic(SysIdRoutine.Direction.kReverse),
        // Commands.print("-- SysIdCompleted! -- ")
        // );
    }
}
