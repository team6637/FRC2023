// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnCommandNew extends CommandBase {
  
  Swerve swerve;

  private double setpoint;
  private double currentAngle;
  private double output;
  private double tolerance = 3;
  private double velocityTolerance = 6;
  private double maxVelocity = 5.0;
  private double maxAcceleration = 2.0;

  PidConfig pidConfig = new PidConfig("turn command", 0.025, 0.0, 0.0, 2.0, 4.0, Constants.CommandConstants.turnIsTunible);

  // TODO CHANGE TO PROFILED PID
  ProfiledPIDController pid = new ProfiledPIDController(pidConfig.getKp(), pidConfig.getKi(), pidConfig.getKd(), new TrapezoidProfile.Constraints(pidConfig.getMaxVelocity(), pidConfig.getMaxAcceleration()));

  public TurnCommandNew(Swerve swerve, double setpoint) {
    this.swerve = swerve;
    this.setpoint = setpoint;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid.setTolerance(tolerance, velocityTolerance);
    pid.enableContinuousInput(-180, 180);
    if(Constants.CommandConstants.turnIsTunible) {
      tolerance = SmartDashboard.getNumber("turn command tolerance", tolerance);
      velocityTolerance = SmartDashboard.getNumber("turn command velocity tolerance", velocityTolerance);
      setpoint = SmartDashboard.getNumber("turn command setpoint", setpoint);
      maxVelocity = SmartDashboard.getNumber("turn command max velocity", maxVelocity);
      maxAcceleration = SmartDashboard.getNumber("turn command max acceleration", maxAcceleration);

      SmartDashboard.putNumber("turn command tolerance", tolerance);
      SmartDashboard.putNumber("turn command velocity tolerance", velocityTolerance);
      SmartDashboard.putNumber("turn command setpoint", setpoint);
      SmartDashboard.putNumber("turn command max velocity", maxVelocity);
      SmartDashboard.putNumber("turn command max acceleration", maxAcceleration);
    }
  }

  @Override
  public void execute() {
    pidConfig.updateFromSmartDashboard();
    if(Constants.CommandConstants.turnIsTunible) {
      SmartDashboard.putBoolean("turn command pid is at setpoint", pid.atSetpoint());

      tolerance = SmartDashboard.getNumber("turn command tolerance", tolerance);
      velocityTolerance = SmartDashboard.getNumber("turn command velocity tolerance", velocityTolerance);
      setpoint = SmartDashboard.getNumber("turn command setpoint", setpoint);
      maxVelocity = SmartDashboard.getNumber("turn command max velocity", maxVelocity);
      maxAcceleration = SmartDashboard.getNumber("turn command max acceleration", maxAcceleration);
      
      pid.setP(pidConfig.getKp());
      pid.setTolerance(tolerance, velocityTolerance);
    }

    currentAngle = swerve.getPose().getRotation().getDegrees();
    output = pid.calculate(currentAngle, setpoint);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, output);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);


  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    //return pid.atSetpoint();
    return false;
  }
}
