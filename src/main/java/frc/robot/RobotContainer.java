// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

    private final Swerve m_swerve = new Swerve();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    //private final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem();
    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    //private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();

    private final Joystick m_joystick = new Joystick(0);


    public RobotContainer() {
        // setup auton dropdown
        m_swerve.setDefaultCommand(
            new SwerveTeleopCommand(
                m_swerve,
                () -> m_joystick.getY(),
                () -> m_joystick.getX(),
                () -> m_joystick.getTwist()
            )
        );
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // new JoystickButton(m_joystick, 6).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        // new JoystickButton(m_joystick, 9).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        new JoystickButton(m_joystick, 7).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        new JoystickButton(m_joystick, 8).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        new JoystickButton(m_joystick, 2).whileTrue(new InstantCommand(() -> m_armSubsystem.setSetpoint(0.0)));

        // new JoystickButton(m_joystick, 5).whileTrue(new RunCommand(() -> m_gripperSubsystem.open())).onFalse(new InstantCommand(() -> m_gripperSubsystem.stop()));

        // new JoystickButton(m_joystick, 10).whileTrue(new RunCommand(() -> m_gripperSubsystem.close())).onFalse(new InstantCommand(() -> m_gripperSubsystem.stop()));

    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
