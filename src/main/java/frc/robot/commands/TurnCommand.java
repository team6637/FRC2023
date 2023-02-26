// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnCommand extends CommandBase {
  Swerve swerve;
  double setpoint;
  double kp = 0.019;
  double error;
  int timer = 0;

  public TurnCommand(Swerve swerve, double setpoint) {
    this.swerve = swerve;
    this.setpoint = setpoint;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = swerve.getPose().getRotation().getDegrees();

    if(currentAngle > -180.0 && currentAngle < -90.0) {
      currentAngle = currentAngle + 360.0;
    }

    error = setpoint - currentAngle;

    if(error < 2.0) timer++;

    double output = kp * error;

    if (Math.abs(output) > 0.29) {
      output = 0.29 * Math.signum(output);
    }

    output = output * Constants.Swerve.maxAngularVelocity;

    SmartDashboard.putNumber("turn command final output", output);
    SmartDashboard.putNumber("turn command timer", timer);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, output);
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
    return timer > 50;
  }
}
