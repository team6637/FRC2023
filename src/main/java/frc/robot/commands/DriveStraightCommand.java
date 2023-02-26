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
  boolean isBackwards;

  double angleKp = 0.01;
  PIDController anglePid = new PIDController(angleKp, 0.0, 0.0);
  double initialHeading;
  
  public DriveStraightCommand(Swerve swerve, double targetX, boolean isBackwards) {
    this.swerve = swerve;
    this.targetX = targetX;
    this.isBackwards = isBackwards;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("drive straight kp", kp);
    pid.setTolerance(.01);
    this.setpoint = swerve.getPose().getX() + targetX;
    initialHeading = swerve.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // kp = SmartDashboard.getNumber("drive straight kp", kp);
    // pid.setP(kp);
    double currentX = swerve.getPose().getX();
    double output = pid.calculate(currentX, setpoint);

    SmartDashboard.putNumber("drive straight output", output);
    SmartDashboard.putNumber("drive straight currentX", currentX);
    SmartDashboard.putNumber("drive straight error", pid.getPositionError());
    SmartDashboard.putNumber("drive straight setpoint", setpoint);

    output = output + 0.1 * Math.signum(output);

    if (Math.abs(output) > 0.2) {
      output = 0.2 * Math.signum(output);
    }

    output = output * Constants.Swerve.maxSpeed;
    if(isBackwards) output = output * -1.0;

    SmartDashboard.putNumber("drive straight final output", output);


    double zOutput = anglePid.calculate(swerve.getHeading(), initialHeading);
    if (Math.abs(zOutput) > 0.3) {
      zOutput = 0.3 * Math.signum(zOutput);
    }
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;


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
