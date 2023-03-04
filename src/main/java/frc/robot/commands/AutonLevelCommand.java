// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutonLevelCommand extends CommandBase {

  double rollKp = 0.017;
  double yawKp = 0.01;
  double maxSpeed;
  double slowMaxSpeed = 0.08;
  boolean hasGoneBelowZeroOnce;
  boolean hasGoneBelowZeroTwice;
  int counter;

  Swerve swerve;
  PIDController rollPID = new PIDController(rollKp, 0.0, 0.0);
  PIDController yawPID = new PIDController(rollKp, 0.0, 0.0);
  double initialHeading;

  public AutonLevelCommand(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("level roll rollKp", rollKp);    
    SmartDashboard.putNumber("level yaw pid", yawKp);
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

    double xOutput = rollPID.calculate(swerve.getRoll(), 0.0 + swerve.initialRoll);

    if (Math.abs(xOutput) > maxSpeed) {
      xOutput = maxSpeed * Math.signum(xOutput);
    }

    xOutput = -xOutput * Constants.Swerve.maxSpeed;

    // yawKp = SmartDashboard.getNumber("level yawKp", yawKp);
    // yawPID.setP(yawKp);

    double zOutput = yawPID.calculate(swerve.getHeading(), initialHeading);
    if (Math.abs(zOutput) > 0.2) {
      zOutput = 0.2 * Math.signum(zOutput);
    }
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;


    SmartDashboard.putNumber("level error", rollPID.getPositionError());
    SmartDashboard.putNumber("level output", xOutput);

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

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasGoneBelowZeroOnce;
  }
}
