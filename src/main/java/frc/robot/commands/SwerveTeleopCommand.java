// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveTeleopCommand extends CommandBase {
  
  private final Swerve swerve;
  private DoubleSupplier x, y, z, visionOffset, driveInversionMultiplier;
  private SlewRateLimiter xLimiter, yLimiter, zLimiter;
  boolean isAuton;

  public SwerveTeleopCommand(Swerve swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, DoubleSupplier visionOffset, DoubleSupplier driveInversionMultiplier, boolean isAuton) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.z = z;
    this.visionOffset = visionOffset;
    this.driveInversionMultiplier = driveInversionMultiplier;

    xLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelerationMetersPerSecond);
    yLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelerationMetersPerSecond);
    zLimiter = new SlewRateLimiter(Constants.Swerve.maxAccelerationRadiansPerSecond);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("joystick raw x", this.x.getAsDouble());
    SmartDashboard.putNumber("joystick raw y", this.y.getAsDouble());
    SmartDashboard.putNumber("joystick raw z", this.z.getAsDouble());
    SmartDashboard.putNumber("vision offset", this.visionOffset.getAsDouble());

    // adjust for joystick drift
    double xOutput = modifyAxis(this.x.getAsDouble(), 0.1) * driveInversionMultiplier.getAsDouble();
    double yOutput = modifyAxis(this.y.getAsDouble(), 0.2) * driveInversionMultiplier.getAsDouble();

    //double xOutput = modifyAxis(this.x.getAsDouble(), 0.1);
    //double yOutput = modifyAxis(this.y.getAsDouble(), 0.2);

    double zOutput = modifyAxis(this.z.getAsDouble(), 0.1);

    yOutput = yOutput + visionOffset.getAsDouble();
    if (Math.abs(yOutput) > 1) {
      yOutput = 1 * Math.signum(yOutput);
    }
    zOutput = zOutput * Constants.Swerve.maxTurnMultiplier;

    xOutput = xOutput * Constants.Swerve.maxSpeed;
    yOutput = yOutput * Constants.Swerve.maxSpeed;
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;

    // limit fast joystick changes
    // xOutput = xLimiter.calculate(xOutput);
    // yOutput = yLimiter.calculate(yOutput);
    // zOutput = zLimiter.calculate(zOutput);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, zOutput);

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
    return isAuton ? visionOffset.getAsDouble() < 1 : false;
  }

  private static double modifyAxis(double value, double deadband) {
      // Deadband - ignore really low numbers
      value = MathUtil.applyDeadband(value, deadband);
      //value = deadband(value, deadband);

      // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
      value = Math.copySign(value * value, value);

      return value;
  }
}
