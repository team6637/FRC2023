// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_ARM_PORT, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_PORT, MotorType.kBrushless);
  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(0);

  private final double maxAngle = Constants.ArmConstants.maxAngle;
  private final double minAngle = Constants.ArmConstants.minAngle;
  private final double angleOffset = 110.7;
  private double kp = 0.01;
  private double minPowerAtLevel = 0.025;
  private double minPowerAtExtended = 0.03;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private double setpoint = minAngle;
  private double setpointIncrementer = 0.5;
  private double motorOutput = 0.0;
  private double maxPIDSpeed = 0.3;
  private double downSpeed = -0.2;
  private final ExtenderSubsystem extenderSubsystemReference;

  // Settings
  private boolean usingPID = true;
  private boolean settingMinLevel = true;

  public ArmSubsystem(ExtenderSubsystem extenderSubsystem) {
    pid.setTolerance(5.0);

    this.extenderSubsystemReference = extenderSubsystem;
    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);

    if (Constants.ArmConstants.isTunable) {
      SmartDashboard.putNumber("arm min power to hold level", minPowerAtLevel);
      SmartDashboard.putNumber("arm kp", kp);
      SmartDashboard.putNumber("arm setpoint", setpoint);
      SmartDashboard.putNumber("down speed", downSpeed);
    }
    setSetpoint(minAngle);
  }

  public void raise() {

    if (usingPID) {
      double tempSetpoint = setpoint += setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = 0.2;
    }
  }

  public void lower() {
    if (usingPID) {
      double tempSetpoint = setpoint -= setpointIncrementer;
      setSetpoint(tempSetpoint);
    } else {
      motorOutput = -0.2;
    }
  }

  public void stop() {
    if (!usingPID) {
      motorOutput = 0.0;
    }
  }

  public double getFeedForward() {
    return Math.cos(Math.toRadians(getDegrees()))*getMinPower();
  }

  public double getMinPower() {
    double percentExtended = extenderSubsystemReference.getPosition() / Constants.ExtenderConstants.EXTENDER_MAX_EXTENSION;
    return (minPowerAtExtended - minPowerAtLevel) * percentExtended + minPowerAtLevel;
  }

  public double getPidOutput() {
    double speed = pid.calculate(getDegrees(), setpoint) + getFeedForward();
    if (setpoint + 10 < getDegrees()) {
      downSpeed = SmartDashboard.getNumber("down speed", downSpeed);
      return downSpeed;
    }
    if (speed >= maxPIDSpeed) {
      return maxPIDSpeed;
    }
    return speed;
  }

  public double getDegrees() {
    return (throughboreEncoder.getAbsolutePosition() -0.5)*-1.0*360.0 - angleOffset;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint > maxAngle) {
      setpoint = maxAngle;
    } else if (newSetpoint < minAngle) {
      setpoint = minAngle;
    } else {
      setpoint = newSetpoint;
    }
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }
  
  @Override
  public void periodic() {
    if (Constants.ArmConstants.isTunable) {
      SmartDashboard.putNumber("arm throughbore encoder value", getDegrees());
      SmartDashboard.putBoolean("arm within range up", getDegrees() >= maxAngle);
      SmartDashboard.putBoolean("arm within range down", getDegrees() <= minAngle);
      SmartDashboard.putNumber("arm power output", getPidOutput());
      SmartDashboard.putNumber("arm feed forward", getFeedForward());
      SmartDashboard.putNumber("arm Math.cos*getdegrees", Math.cos(Math.toRadians(getDegrees())));
      minPowerAtLevel = SmartDashboard.getNumber("arm min power to hold level", minPowerAtLevel);
      kp = SmartDashboard.getNumber("arm kp", kp);
      pid.setP(kp);
      SmartDashboard.putNumber("arm position error", pid.getPositionError());
      SmartDashboard.putNumber("arm min power", getMinPower());
    }

    if (usingPID) {
      leftMotor.set(getPidOutput());
    } else {
      if (getDegrees() >= maxAngle) motorOutput = 0.0;
      if (getDegrees() <= minAngle) motorOutput = 0.0;
      if (settingMinLevel) motorOutput = minPowerAtLevel;
      leftMotor.set(motorOutput);
    }
  }
}