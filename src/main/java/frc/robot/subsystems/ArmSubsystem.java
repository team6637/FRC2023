// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.SettableBeanProperty;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_ARM_PORT, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_PORT, MotorType.kBrushless);
  private final DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(0);

  private final double maxAngle = 40.0;
  private final double minAngle = -63.0;
  private final double angleOffset = 215.0;
  private double kp = 0.0;
  private double minPowerAtLevel = 0.0;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private double setpoint;
  private boolean usingPID = false;
  private double setpointIncrementer = 0.5;
  private double motorOutput = 0.0;
  private boolean settingMinLevel = true;

  public ArmSubsystem() {
    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);
  
    SmartDashboard.putNumber("arm min power to hold level", minPowerAtLevel);
    SmartDashboard.putNumber("arm kp", kp);
  }

  public void raise() {
    if (usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
      motorOutput = 0.2;
    }
  }

  public void lower() {
    if (usingPID) {
      setSetpoint(setpoint -= setpointIncrementer);
    } else {
      motorOutput = -0.2;
    }
  }

  public void stop() {
    motorOutput = 0.0;
  }

  public double getFeedForward() {
    return Math.cos(Math.toRadians(getDegrees()))*minPowerAtLevel;
  }

  public double getPidOutput() {
    return pid.calculate(getDegrees(), setpoint)+getFeedForward();
  }

  public double getDegrees() {
    return throughboreEncoder.get()*360.0-angleOffset;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint > maxAngle) {
      newSetpoint = maxAngle;
    } else if (newSetpoint < minAngle) {
      newSetpoint = minAngle;
    } else {
      setpoint = newSetpoint;
    }
    SmartDashboard.putNumber("arm setpoint", setpoint);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm throughbore encoder value", -getDegrees() - 130);
    SmartDashboard.putBoolean("arm within range up", getDegrees() >= maxAngle);
    SmartDashboard.putBoolean("arm within range down", getDegrees() <= minAngle);
    SmartDashboard.putNumber("arm power output", getPidOutput());
    setSetpoint(SmartDashboard.getNumber("arm setpoint", setpoint));
    SmartDashboard.putNumber("arm feed forward", getFeedForward());
    SmartDashboard.putNumber("arm Math.cos*getdegrees", Math.cos(Math.toRadians(getDegrees())));
    minPowerAtLevel = SmartDashboard.getNumber("arm min power to hold level", minPowerAtLevel);
    kp = SmartDashboard.getNumber("arm kp", kp);
    pid.setP(kp);

    if (usingPID) {
      leftMotor.set(getPidOutput());
    } else {
      if (settingMinLevel) motorOutput = minPowerAtLevel;
      leftMotor.set(motorOutput);
    }

    

  }
}
