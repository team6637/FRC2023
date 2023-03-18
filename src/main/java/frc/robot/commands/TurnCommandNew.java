// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnCommandNew extends CommandBase {
  
  Swerve swerve;

  private double setpoint;
  private double currentAngle;
  private double output, pidOutput, ffOutput;
  private double tolerance = 2;
  private double velocityTolerance = 6;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  PidConfig pidConfig = new PidConfig("turn command", 0.01, 0.0, 0.0, Constants.CommandConstants.turnIsTunable);

  PIDController pid = new PIDController(pidConfig.getKp(), pidConfig.getKi(), pidConfig.getKd());

  public TurnCommandNew(Swerve swerve, double setpoint) {
    this.swerve = swerve;
    this.setpoint = setpoint;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    pid.enableContinuousInput(-180, 180);
    pid.setTolerance(tolerance, velocityTolerance);
  }

  @Override
  public void execute() {

    currentAngle = swerve.getPose().getRotation().getDegrees();

    pidOutput = pid.calculate(currentAngle, setpoint);

    pidOutput = MathUtil.clamp(pidOutput, -0.4, 0.4);

    ffOutput = feedforward.calculate(output * Constants.Swerve.maxSpeed);

    output *= Constants.Swerve.maxAngularVelocity;
    output = output + ffOutput;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, output);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);

    SmartDashboard.putBoolean("turn command at setpoint", pid.atSetpoint());
    SmartDashboard.putNumber("turn command pid output", output);
    SmartDashboard.putNumber("turn command error", pid.getPositionError());
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
