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


public class DriveStraightCommand extends CommandBase {
  
  Swerve swerve;
  double setpoint;
  double targetX;
  double kp = 0.5;
  PIDController pid = new PIDController(kp, 0.0, 0.0);
  
  public DriveStraightCommand(Swerve swerve, double targetX) {
    this.swerve = swerve;
    this.targetX = targetX;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("drive straight kp", kp);
    pid.setTolerance(.01);
    this.setpoint = swerve.getPose().getX() + targetX;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kp = SmartDashboard.getNumber("drive straight kp", kp);
    pid.setP(kp);
    double currentX = swerve.getPose().getX();
    double output = pid.calculate(currentX, setpoint);

    SmartDashboard.putNumber("drive straight output", output);
    SmartDashboard.putNumber("drive straight currentX", currentX);
    SmartDashboard.putNumber("drive straight error", pid.getPositionError());
    SmartDashboard.putNumber("drive straight setpoint", setpoint);

    output = output + 0.1 * Math.signum(output);

    if (Math.abs(output) > 0.3) {
      output = 0.3 * Math.signum(output);
    }

    output = -output * Constants.Swerve.maxSpeed;

    SmartDashboard.putNumber("drive straight final output", output);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(output, 0.0, 0.0);
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
    return pid.atSetpoint();
  }
}
