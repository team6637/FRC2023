// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  public ArmSubsystem() {
    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);
  
  }

  private final double maxAngle = 40.0;
  private final double minAngle = -60.0;
  private final double angleOffset = 215.0;
  private final double kp = 0.04;
  private final double minPowerAtLevel = 0.3;
  private final PIDController pid = new PIDController(kp, 0.0, 0.0);
  private double setpoint;

  public void raise() {
    leftMotor.set(0.2);
    }

  public void lower() {
    leftMotor.set(-0.2);
  }

  public void stop() {
    leftMotor.set(0.0);
  }


  public double getFeedForward() {
    return  Math.cos(Math.toRadians(getDegrees()))*minPowerAtLevel;
  }

  public double getPower() {
    return pid.calculate(getDegrees(), setpoint)+getFeedForward();
  }

  public double getDegrees() {
    return throughboreEncoder.get()*360.0-angleOffset;
  }

  public void EncoderSubsystem() {
    setpoint = getDegrees();
    SmartDashboard.putNumber("Setpoint", setpoint);
  }

  public void raiseSetpoint() {
    setpoint += 0.5;
  }
  public void lowerSetpoint() {
    setpoint -= 0.5;
  }
  public void setSetpoint(int point) {
    setpoint = point;
  }
  
  @Override
  public void periodic() {
    getDegrees();
    SmartDashboard.putNumber("Throughbore encoder value", -getDegrees() - 130);
    SmartDashboard.putBoolean("Within range up", getDegrees() >= maxAngle);
    SmartDashboard.putBoolean("Within range down", getDegrees() <= minAngle);
    SmartDashboard.putNumber("Power output", getPower());
    setpoint = SmartDashboard.getNumber("Setpoint", setpoint);
    SmartDashboard.putNumber("Feed forward", getFeedForward());
    SmartDashboard.putNumber("Math.cos*getdegrees", Math.cos(Math.toRadians(getDegrees())));
    
  }
}
