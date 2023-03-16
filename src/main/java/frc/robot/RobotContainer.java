// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.TurnCommandNew;
import frc.robot.commands.autos.AutonChargeStationCommand2;
import frc.robot.commands.autos.AutonOneCommand;
import frc.robot.commands.autos.AutonThreeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {

    private final Swerve m_swerve = new Swerve();
    private final ExtenderSubsystem m_extenderSubsystem = new ExtenderSubsystem();
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_extenderSubsystem);
    public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
    private final GripperSubsystem m_gripperSubsystem = new GripperSubsystem();
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
    //private final DistanceSensorSubsystem m_distanceSensorSubsystem = new DistanceSensorSubsystem();
    private final Joystick driverStick = new Joystick(0);
    private final Joystick controlStick = new Joystick(1);
    private String mode = "cube";

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {

        // setup auton dropdown
        m_swerve.setDefaultCommand(
            new SwerveTeleopCommand(
                m_swerve,
                () -> -driverStick.getY(),
                () -> -driverStick.getX(),
                () -> -driverStick.getTwist(),
                () -> m_limelightSubsystem.getTx(),
                () -> driveInversionMultiplier(),
                false
            )
        );
        
        m_chooser.setDefaultOption("Auton 1 (Main)", new AutonOneCommand(m_swerve, m_armSubsystem, m_extenderSubsystem, m_gripperSubsystem, m_limelightSubsystem));
        //m_chooser.addOption("Auton 2 Charge Station", new AutonChargeStationCommand(m_swerve, m_armSubsystem, m_extenderSubsystem, m_gripperSubsystem, m_limelightSubsystem));
        m_chooser.addOption("Auton 2 Charge Station)", new AutonChargeStationCommand2(m_swerve, m_armSubsystem, m_extenderSubsystem, m_gripperSubsystem, m_limelightSubsystem));
        m_chooser.addOption("Auton 3", new AutonThreeCommand(m_swerve, m_armSubsystem, m_extenderSubsystem, m_gripperSubsystem, m_limelightSubsystem));
        m_chooser.addOption("Turn (You realy just want to turn?)", new TurnCommandNew(m_swerve, 0));

        SmartDashboard.putData(m_chooser);

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        // EXTENDER
        new JoystickButton(driverStick, 6).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        new JoystickButton(driverStick, 9).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract())).onFalse(new InstantCommand(() -> m_extenderSubsystem.stop()));

        // ARM (DRIVER STICK)
        new JoystickButton(driverStick, 5).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        new JoystickButton(driverStick, 10).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        // ARM (CONTROL STICK)
        new JoystickButton(controlStick, 8).whileTrue(new RunCommand(() -> m_armSubsystem.raise())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));

        //new JoystickButton(controlStick, 10).whileTrue(new RunCommand(() -> m_armSubsystem.lower())).onFalse(new InstantCommand(() -> m_armSubsystem.stop()));


        // SET LOW
        new JoystickButton(controlStick, 11).onTrue(new SequentialCommandGroup(
            new InstantCommand(() ->  m_extenderSubsystem.setSetpoint(0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> m_extenderSubsystem.atSetpoint()),
            new InstantCommand(() -> m_gripperSubsystem.setSetpoint(20)),
            new InstantCommand(() -> m_armSubsystem.setSetpoint(Constants.ArmConstants.minAngle)),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.fullOpen))
        ));

        // SET MID
        new JoystickButton(controlStick, 9).onTrue(new SequentialCommandGroup(
            new ConditionalCommand(
                new InstantCommand(() ->  m_armSubsystem.setSetpoint(20)), 
                new InstantCommand(() ->  m_armSubsystem.setSetpoint(8)), 
                ()-> getMode() == "cone"
            ),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_extenderSubsystem.setSetpoint(17.5))
        ));

        // SET HIGH
        new JoystickButton(controlStick, 7).onTrue(new SequentialCommandGroup(
            new ConditionalCommand(
                new InstantCommand(() ->  m_armSubsystem.setSetpoint(30)), 
                new InstantCommand(() ->  m_armSubsystem.setSetpoint(19)), 
                ()-> getMode() == "cone"
            ),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> m_armSubsystem.atSetpoint()),
            new InstantCommand(() -> m_extenderSubsystem.setSetpoint(150))
        ));


        // SET LOW WITH OBJECT
        new JoystickButton(controlStick, 12).onTrue(new SequentialCommandGroup(
            new InstantCommand(() ->  m_extenderSubsystem.setSetpoint(0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> m_extenderSubsystem.atSetpoint()),
            new InstantCommand(() -> m_armSubsystem.setSetpoint(-35.0))
        ));

        // SET MID FOR SHELF
        new JoystickButton(controlStick, 10).onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> m_armSubsystem.setSetpoint(14.89)),
            new InstantCommand(() ->  m_extenderSubsystem.setSetpoint(0))
        ));



        // GRIPPER
        new JoystickButton(driverStick, 1).whileTrue(new RunCommand(() -> m_gripperSubsystem.closeGripper()));

        new JoystickButton(driverStick, 3).whileTrue(
            new ConditionalCommand(
                new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.fullOpen)),
                new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.fullOpenWhenExtended)),
                ()->m_armSubsystem.getDegrees() < -10.0
            )
        );

        new JoystickButton(driverStick, 2).onTrue(new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.closeCone)));

        new JoystickButton(driverStick, 4).onTrue(new InstantCommand(() -> m_gripperSubsystem.setSetpoint(Constants.GripperConstants.closeCube)));



        // LED
        new JoystickButton(controlStick, 3).whileTrue(new InstantCommand(()->setModeCone()));
        new JoystickButton(controlStick, 4).whileTrue(new InstantCommand(()->setModeCube()));



        // LIMELIGHT
        // Object Detection
        new JoystickButton(driverStick, 8).whileTrue(new RunCommand(() -> m_limelightSubsystem.setVisionMode("object"))).onFalse(new InstantCommand(() -> m_limelightSubsystem.setVisionMode("off")));

        // April Tag & Reflective Tape Detection
        new JoystickButton(driverStick, 7).whileTrue(new RunCommand(() -> m_limelightSubsystem.setVisionMode(mode))).onFalse(new InstantCommand(() -> m_limelightSubsystem.setVisionMode("off")));

        //Test balence
        // new JoystickButton(controlStick, 8).whileTrue(new AutonLevelCommand(m_swerve));

        // POV fine control
        new POVButton(driverStick, 0).whileTrue(new RunCommand(() -> m_extenderSubsystem.extend()));

        new POVButton(driverStick, 180).whileTrue(new RunCommand(() -> m_extenderSubsystem.retract()));

        new POVButton(driverStick, 270).whileTrue(new SwerveTeleopCommand(
            m_swerve,
            () -> 0.0,
            () -> 0.45,
            () -> 0.0,
            () -> 0.0,
            () -> driveInversionMultiplier(),
            false
        ));

        new POVButton(driverStick, 90).whileTrue(new SwerveTeleopCommand(
            m_swerve,
            () -> 0.0,
            () -> -0.45,
            () -> 0.0,
            () -> 0.0,
            () -> driveInversionMultiplier(),
            false
        ));

        new POVButton(controlStick, 0).whileTrue(new RunCommand(() -> m_armSubsystem.raise()));

        new POVButton(controlStick, 180).whileTrue(new RunCommand(() -> m_armSubsystem.lower()));

    }

    public Double driveInversionMultiplier() {
        return 1.0;
        // if(driverStick.getThrottle() < 0) { // new joystick up is -1.0
        //     return 1.0;
        // } else {
        //     return -1.0;
        // } 
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }


    // GAME OBJECT STATE
    public void setModeCube() {
        mode = "cube";
        m_ledSubsystem.turnLEDpurple();
    }
    public void setModeCone() {
        mode = "cone";
        m_ledSubsystem.turnLEDyellow();
    }
    public String getMode() {
        return mode;
    }
}
