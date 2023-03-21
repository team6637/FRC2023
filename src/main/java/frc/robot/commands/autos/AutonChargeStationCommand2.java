// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoLevelCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.TurnWheelsCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class AutonChargeStationCommand2 extends SequentialCommandGroup {
    public AutonChargeStationCommand2(Swerve swerve, ArmSubsystem arm, ExtenderSubsystem extender, GripperSubsystem gripper, LimelightSubsystem vision) {

        addCommands(
            // SET ROBOT POSE
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(1.87, 4.44, new Rotation2d(Math.toRadians(180.0))))),

            // LIFT ARM
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.closeCube)),
            new InstantCommand(() ->  arm.setSetpoint(14.0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> extender.setSetpoint(120)),
            
            // DRIVE FORWARD
            new DriveStraightCommand(swerve, -0.1, true, 0.0),

            // DROP OBJECT
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.autonSetSetpoint(Constants.GripperConstants.fullOpenWhenExtended)),
            new WaitCommand(0.5),

            // DO 2 THINGS AT ONCE
            new ParallelCommandGroup(

                // DRIVE ONTO CHARGE STATION
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new DriveStraightCommand(swerve, 2.3, true, 0.2)
                ),

                // RESET ARM
                new SequentialCommandGroup(
                    new InstantCommand(() ->  extender.setSetpoint(0)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> extender.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(20)),
                    new InstantCommand(() -> arm.setSetpoint(Constants.ArmConstants.minAngle)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> arm.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.fullOpen))
                )
            ),

            // DRIVE UNTIL PLATFORM GOES PAST LEVEL
            new AutoLevelCommand(swerve),

            // DRIVE BACK SET DISTANCE IN METERS (.3M = 1 FT)
            new DriveStraightCommand(swerve, -0.19, true, -0.1),

            new TurnWheelsCommand(swerve)
        );
    }

}
