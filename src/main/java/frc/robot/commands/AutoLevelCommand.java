// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoLevelCommand extends CommandBase {

  Swerve swerve;

  double rollKp = 0.017;
  double yawKp = 0.017;
  double maxSpeed;
  double slowMaxSpeed = 0.08;
  double initialHeading;
  boolean hasGoneBelowZeroOnce;
  boolean hasGoneBelowZeroTwice;
  int counter;

  PidConfig rollPidConfig = new PidConfig("AutonLevel-roll", rollKp, Constants.CommandConstants.levelRollIsTunible);
  PidConfig yawPidConfig = new PidConfig("AutonLevel-yaw", yawKp, Constants.CommandConstants.levelYawIsTunible);
  PIDController rollPID = new PIDController(rollPidConfig.getKp(), rollPidConfig.getKi(), rollPidConfig.getKd());
  PIDController yawPID = new PIDController(yawPidConfig.getKp(), yawPidConfig.getKi(), yawPidConfig.getKd());

  public AutoLevelCommand(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (Constants.CommandConstants.levelRollIsTunible) {
      SmartDashboard.putNumber("level roll rollKp", rollKp);    
    }
    if (Constants.CommandConstants.levelYawIsTunible) {
      SmartDashboard.putNumber("level yaw pid", yawKp);
    }

    initialHeading = swerve.getHeading();
    counter = 0;
    hasGoneBelowZeroOnce = false;
    hasGoneBelowZeroTwice = false;
    maxSpeed = 0.12;
  }

  @Override
  public void execute() {
    //rollKp = SmartDashboard.getNumber("level  rollKp", rollKp);
    //rollPID.setP(rollKp);
    double roll = swerve.getRoll();
    if(Math.abs(roll) < 1) hasGoneBelowZeroOnce = true;
    if(hasGoneBelowZeroOnce) maxSpeed = slowMaxSpeed;

    if(hasGoneBelowZeroOnce) {
      counter++;
      if(counter > 250) {
        if(Math.abs(roll) < 7.5) hasGoneBelowZeroTwice = true;
      }
    }

    //Calculate roll
    double xOutput = rollPID.calculate(swerve.getRoll(), 0.0 + swerve.initialRoll);
    if (Math.abs(xOutput) > maxSpeed) {
      xOutput = maxSpeed * Math.signum(xOutput);
    }
    xOutput = -xOutput * Constants.Swerve.maxSpeed;
    
    if (Constants.CommandConstants.levelRollIsTunible) {
      SmartDashboard.putNumber("level roll error", rollPID.getPositionError());
      SmartDashboard.putNumber("level roll output", xOutput);

      rollPidConfig.updateFromSmartDashboard();
    }

    //Calculate yaw
    double zOutput = yawPID.calculate(swerve.getHeading(), initialHeading);
    if (Math.abs(zOutput) > 0.2) {
      zOutput = 0.2 * Math.signum(zOutput);
    }
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;

    if (Constants.CommandConstants.levelYawIsTunible) {
      SmartDashboard.putNumber("level yaw error", yawPID.getPositionError());
      SmartDashboard.putNumber("level yaw output", zOutput);

      yawPidConfig.updateFromSmartDashboard();
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, 0.0, zOutput);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // spin the wheels
    // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.1, 0.0);
    // SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    // swerve.setModuleStates(moduleStates);

    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return hasGoneBelowZeroOnce;
  }
}
