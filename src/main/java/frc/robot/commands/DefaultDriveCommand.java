// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;

  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final DoubleSupplier z;
  private final DoubleSupplier visionOffset;
  private double turnMultiplier = 0.5;

  public DefaultDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, DoubleSupplier visionOffset) {
    this.m_driveSubsystem = driveSubsystem;
    this.x = x;
    this.y = y;
    this.z = z;
    this.visionOffset = visionOffset;

    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    drive_robot_relative();
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public void drive_field_relative() {
    m_driveSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            x.getAsDouble(),
            y.getAsDouble() + visionOffset.getAsDouble(),
            z.getAsDouble() * turnMultiplier,
            m_driveSubsystem.getGyroscopeRotation()
        )
    );
  }

  public void drive_robot_relative() {
    m_driveSubsystem.drive(new ChassisSpeeds(
        x.getAsDouble(),
        y.getAsDouble() + visionOffset.getAsDouble(),
        z.getAsDouble() * turnMultiplier
    ));
  }
}












































































































//forrest was herer