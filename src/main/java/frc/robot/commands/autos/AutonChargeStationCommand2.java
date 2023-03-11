// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AutonLevelCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.TurnWheelsCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class AutonChargeStationCommand2 extends SequentialCommandGroup {
    public AutonChargeStationCommand2(Swerve swerve, ArmSubsystem arm, ExtenderSubsystem extender, GripperSubsystem gripper, LimelightSubsystem vision) {

        addCommands(

            new TurnWheelsCommand(swerve),
            new WaitCommand(20),

            // SET ROBOT POSE
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.87, 4.44, new Rotation2d(Math.toRadians(180.0))))),

            // LIFT ARM
            new InstantCommand(() -> gripper.setSetpoint(Constants.Gripper.closeCube)),
            new InstantCommand(() ->  arm.setSetpoint(14.0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> extender.setSetpoint(120)),
            
            // DRIVE FORWARD
            new DriveStraightCommand(swerve, -0.1, true, 0.0),

            // DROP OBJECT
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.autonSetSetpoint(Constants.Gripper.fullOpenWhenExtended)),
            new WaitCommand(0.5),



            // DRIVE TO OBJECT
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new DriveStraightCommand(swerve, 2.3, true, 0.2)
                ),

                // drive over
                //new DriveStraightCommand(swerve, 1.8, true, 0.1),

                new SequentialCommandGroup(
                    // RESET ARM
                    new InstantCommand(() ->  extender.setSetpoint(0)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> extender.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(20)),
                    new InstantCommand(() -> arm.setSetpoint(Constants.ArmConstants.minAngle)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> arm.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(Constants.Gripper.fullOpen))
                )
            ),

            new AutonLevelCommand(swerve),


            new DriveStraightCommand(swerve, -0.19, true, -0.1),

            //new WaitCommand(2),

            //new AutonLevelCommand(swerve),




            // stop wheels
            new RunCommand( ()->
                new SwerveTeleopCommand(
                swerve,
                () -> 0.0,
                () -> 0.2,
                () -> 0.0,
                () -> 0.0,
                () -> 1.0,
                true
            )).withTimeout(0.2),

            new InstantCommand(()->swerve.stopModules(), swerve)
            
        );
    }

}
