// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveTeleopCommand extends CommandBase {
  
  private final Swerve swerve;
  private DoubleSupplier x, y, z;
  private SlewRateLimiter xLimiter, yLimiter, zLimiter;

  public SwerveTeleopCommand(Swerve swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.z = z;

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

    // adjust for joystick drift
    double xOutput = modifyAxis(this.x.getAsDouble());
    double yOutput = modifyAxis(this.y.getAsDouble());
    double zOutput = modifyAxis(this.z.getAsDouble());

    xOutput = xOutput * Constants.Swerve.maxSpeed;
    yOutput = yOutput * Constants.Swerve.maxSpeed;
    zOutput = zOutput * Constants.Swerve.maxAngularVelocity;

    // limit fast joystick changes
    // xOutput = xLimiter.calculate(xOutput);
    // yOutput = yLimiter.calculate(yOutput);
    // zOutput = zLimiter.calculate(zOutput);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, zOutput);

    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleState(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static double deadband(double value) {
    double deadband = Constants.Swerve.joystickDeadband;
    if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
  }

  private static double modifyAxis(double value) {
      // Deadband - ignore really low numbers
      value = deadband(value);

      // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
      value = Math.copySign(value * value, value);

      return value;
  }
}
