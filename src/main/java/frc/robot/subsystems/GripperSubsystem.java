// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(Constants.GripperConstants.GRIPPER_MOTOR_PORT, MotorType.kBrushless);
  DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(1);

  private final double encoderOffset = 220.0;
  private double kp = 0.01; // 0.025
  private final boolean usingPID  = true;
  private double setpoint = Constants.GripperConstants.fullOpen;
  private final double setpointIncrementer = 0.8;
  PIDController pid = new PIDController(kp, 0, 0);

  public GripperSubsystem() {
    motor.setInverted(false);

    if (Constants.GripperConstants.isTunable) {
      SmartDashboard.putNumber("gripper kp", kp);
      SmartDashboard.putNumber("gripper setpoint", setpoint);
    }
  }

  public double getEncoderPos() {
    return (throughboreEncoder.getAbsolutePosition())*360.0 - encoderOffset;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint < Constants.GripperConstants.fullOpen) {
      setpoint = Constants.GripperConstants.fullOpen;
    } else if (newSetpoint > Constants.GripperConstants.fullClosed) {
      setpoint = Constants.GripperConstants.fullClosed;
    } else {
      setpoint = newSetpoint;
    }
  }

  public void autonSetSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  public double getPidOutput() {
    double output = pid.calculate(getEncoderPos(), setpoint);
    if(Math.abs(output) > 0.25) output = 0.25 * Math.signum(output);
    return output;
  }

  public void closeGripper() {
    if(usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
      motor.set(0.2);
    }
  }

  public void openGripper() {
    if(usingPID) {
    setSetpoint(setpoint -= setpointIncrementer);
    } else {
      motor.set(-0.2);
    }
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    if (Constants.GripperConstants.isTunable) {
      SmartDashboard.putNumber("gripper encoder", getEncoderPos());
      SmartDashboard.putNumber("gripper setpoint", setpoint);
      kp = SmartDashboard.getNumber("gripper kp", kp);
      pid.setP(kp);
    }

    if(usingPID) {
      motor.set(getPidOutput());
    }
  }
}

