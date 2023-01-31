// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PixySubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final PixySubsystem m_pixySubsystem = new PixySubsystem();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    // private final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem();
    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();

    private final Joystick m_joystick = new Joystick(0);

    // setup auton dropdown
    DefaultDriveCommand defaultDriveCommand = new DefaultDriveCommand(
        m_driveSubsystem,
        () -> modifyAxis(-m_joystick.getY()) * m_driveSubsystem.get_max_velocity(),
        () -> modifyAxis(-m_joystick.getX()) * m_driveSubsystem.get_max_velocity(),
        () -> modifyAxis(-m_joystick.getTwist()) * m_driveSubsystem.get_angular_velocity(),
        () -> m_limelightSubsystem.getTx()
    );

    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(defaultDriveCommand);
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // new JoystickButton(m_joystick, 5).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        // new JoystickButton(m_joystick, 6).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        new JoystickButton(m_joystick, 7).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        new JoystickButton(m_joystick, 9).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband - ignore really low numbers
        value = deadband(value, 0.3);

        // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
        value = Math.copySign(value * value, value);

        return value;
    }
}
