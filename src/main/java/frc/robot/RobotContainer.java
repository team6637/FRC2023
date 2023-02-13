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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

    private final Swerve m_swerve = new Swerve();
    private final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_extenderSubsystem);
    private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
    private final Joystick driverStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);


    public RobotContainer() {
        // setup auton dropdown
        m_swerve.setDefaultCommand(
            new SwerveTeleopCommand(
                m_swerve,
                () -> -driverStick.getY(),
                () -> -driverStick.getX(),
                () -> -driverStick.getTwist(),
                () -> m_limelightSubsystem.getTx(),
                () -> driveInversionMultiplier()
            )
        );
        
        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // EXTENDER
        new JoystickButton(driverStick, 6).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        new JoystickButton(driverStick, 9).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        // ARM
        new JoystickButton(driverStick, 5).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        new JoystickButton(driverStick, 10).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        // RESET
        new JoystickButton(controlStick, 11).onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.closeCone)),
            new InstantCommand(() ->  m_extenderSubsystem.setSetpoint(0)),
            new WaitUntilCommand(() -> m_extenderSubsystem.atSetpoint()),
            new InstantCommand(() -> m_armSubsystem.setSetpoint(Constants.ArmConstants.minAngle)),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.fullOpen))
        ));

        // SET MID
        new JoystickButton(controlStick, 9).onTrue(new SequentialCommandGroup(
            new InstantCommand(() ->  m_armSubsystem.setSetpoint(20)),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_extenderSubsystem.setSetpoint(17.5))
        ));

        // SET HIGH
        new JoystickButton(controlStick, 7).onTrue(new SequentialCommandGroup(
            new InstantCommand(() ->  m_armSubsystem.setSetpoint(26.8)),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_extenderSubsystem.setSetpoint(155))
        ));

        // GRIPPER
        new JoystickButton(driverStick, 1).whileTrue(new RunCommand(() -> m_gripperSubsystem.closeGripper()));

        new JoystickButton(driverStick, 3).whileTrue(new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.fullOpen)));

        new JoystickButton(driverStick, 2).onTrue(new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.closeCone)));

        new JoystickButton(driverStick, 4).onTrue(new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.closeCube)));

        // LIMELIGHT
        // Object Detection
        new JoystickButton(driverStick, 8).whileTrue(new RunCommand(() -> m_limelightSubsystem.setVisionMode("object"))).onFalse(new InstantCommand(() -> m_limelightSubsystem.setVisionMode("off")));

        // April Tag & Reflective Tape Detection
        new JoystickButton(driverStick, 7).whileTrue(new RunCommand(() -> m_limelightSubsystem.setVisionMode("april"))).onFalse(new InstantCommand(() -> m_limelightSubsystem.setVisionMode("off")));

        // POV fine control
        new POVButton(driverStick, 0).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend()));

        new POVButton(driverStick, 180).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract()));

        new POVButton(driverStick, 270).whileTrue(new SwerveTeleopCommand(
            m_swerve,
            () -> 0.0,
            () -> 0.45,
            () -> 0.0,
            () -> 0.0,
            () -> driveInversionMultiplier()
        ));

        new POVButton(driverStick, 90).whileTrue(new SwerveTeleopCommand(
            m_swerve,
            () -> 0.0,
            () -> -0.45,
            () -> 0.0,
            () -> 0.0,
            () -> driveInversionMultiplier()
        ));
    }

    public Double driveInversionMultiplier() {
        if(driverStick.getThrottle() < 0) { // new joystick up is -1.0
            return 1.0;
        } else {
            return -1.0;
        } 
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

}
