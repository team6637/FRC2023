// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonLevelCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class AutonChargeStationCommand extends SequentialCommandGroup {
    public AutonChargeStationCommand(Swerve swerve, ArmSubsystem arm, ExtenderSubsystem extender, GripperSubsystem gripper, LimelightSubsystem vision) {



        addCommands(
            // SET ROBOT POSE
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(2.17, 4.44, new Rotation2d(Math.toRadians(180.0))))),
            new DriveStraightCommand(swerve, 1.73, true),
            new AutonLevelCommand(swerve)
        );
    }

}
