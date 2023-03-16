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


public class DriveStraightCommand extends CommandBase {
  
  Swerve swerve;

  double setpoint;
  double targetX;
  double forwardKp = 0.5;
  double angleKp = 0.01;
  double maxSpeed = 0.2;
  double additionalMaxSpeed;
  double initialHeading;
  double xOutput;
  double zOutput;
  double currentX;
  boolean isBackwards;

  PidConfig forwardPidConfig = new PidConfig("driveStraightForward", forwardKp, Constants.CommandConstants.driveStraightIsTunible);
  PidConfig anglePidConfig = new PidConfig("driveStrightAngle", angleKp, Constants.CommandConstants.driveStraightIsTunible);
  PIDController forwardPid = new PIDController(forwardKp, 0.0, 0.0);
  PIDController anglePid = new PIDController(angleKp, 0.0, 0.0);
  
  public DriveStraightCommand(Swerve swerve, double targetX, boolean isBackwards, double additionalMaxSpeed) {
    this.swerve = swerve;
    this.targetX = targetX;
    this.isBackwards = isBackwards;
    this.maxSpeed += additionalMaxSpeed;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardPid.setTolerance(.01);
    this.setpoint = swerve.getPose().getX() + targetX;
    initialHeading = swerve.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.CommandConstants.driveStraightIsTunible) {
      SmartDashboard.putNumber("driveStraight xOutput", xOutput);
      SmartDashboard.putNumber("driveStraight zOutput", zOutput);
      SmartDashboard.putNumber("driveStraight currentX", currentX);
      SmartDashboard.putNumber("driveStraight error", forwardPid.getPositionError());
      SmartDashboard.putNumber("driveStraight setpoint", setpoint);

      forwardPidConfig.updateFromSmartDashboard();
      anglePidConfig.updateFromSmartDashboard();
    }

    currentX = swerve.getPose().getX();
    xOutput = forwardPid.calculate(currentX, setpoint);

    xOutput = xOutput + 0.1 * Math.signum(xOutput);

    if (Math.abs(xOutput) > maxSpeed) {
      xOutput = maxSpeed * Math.signum(xOutput);
    }

    xOutput = xOutput * Constants.Swerve.maxSpeed;
    if(isBackwards) xOutput = xOutput * -1.0;

    if (Constants.CommandConstants.driveStraightIsTunible) {
      SmartDashboard.putNumber("drive straight final xOutput", xOutput);
    }


    zOutput = anglePid.calculate(swerve.getHeading(), initialHeading);
    if (Math.abs(zOutput) > 0.3) {
      zOutput = 0.3 * Math.signum(zOutput);
    }
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;
    if(!isBackwards) zOutput *= -1.0;


    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, 0.0, zOutput);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return forwardPid.atSetpoint();
  }
}
