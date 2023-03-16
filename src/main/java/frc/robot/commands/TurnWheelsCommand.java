// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnWheelsCommand extends CommandBase {

  Swerve swerve;
  private int counter;

  public TurnWheelsCommand(Swerve swerve) {
    addRequirements(swerve);
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    counter = 0;
  }

  @Override
  public void execute() {
    counter++;
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.2, 0);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return counter > 15;
  }
}
