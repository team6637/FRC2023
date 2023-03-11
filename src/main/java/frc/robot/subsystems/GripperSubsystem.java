// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(Constants.Gripper.GRIPPER_MOTOR_PORT, MotorType.kBrushless);
  DutyCycleEncoder throughboreEncoder = new DutyCycleEncoder(1);

  private double setpoint = Constants.Gripper.fullOpen;

  public static final PidConfig pidConfig = new PidConfig("gripper", 0.01, 0.0, 0.0, true);
  PIDController pid = new PIDController(pidConfig.getKp(), pidConfig.getKi(), pidConfig.getKd());

  //public static final PidConfig pidConfig = new PidConfig("gripper", 0.01, 0.0, 0.005, 1, 1, true);
  //ProfiledPIDController pid = new ProfiledPIDController(pidConfig.getKp(), pidConfig.getKi(), pidConfig.getKd(), new TrapezoidProfile.Constraints(pidConfig.getMaxVelocity(), pidConfig.getMaxAcceleration()));

  public GripperSubsystem() {
    motor.setInverted(false);
  }

  public double getEncoderPos() {
    return (throughboreEncoder.getAbsolutePosition())*360.0 - Constants.Gripper.encoderOffset;
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint < Constants.Gripper.fullOpenWhenExtended) {
      setpoint = Constants.Gripper.fullOpenWhenExtended;
    } else if (newSetpoint > Constants.Gripper.fullClosed) {
      setpoint = Constants.Gripper.fullClosed;
    } else {
      setpoint = newSetpoint;
    }
  }

  public void autonSetSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  public void closeGripper() {
      setSetpoint(setpoint += Constants.Gripper.setpointIncrementer);
  }

  public void openGripper() {
    setSetpoint(setpoint -= Constants.Gripper.setpointIncrementer);
  }

  public void stop() {
    motor.set(0.0);
  }

  @Override
  public void periodic() {
    if (Constants.Gripper.isTunable) {
      pidConfig.updateFromSmartDashboard();

      SmartDashboard.putNumber("gripper encoder", getEncoderPos());
      SmartDashboard.putNumber("gripper setpoint", setpoint);

      pid.setP(pidConfig.getKp());
    }

    double output = pid.calculate(getEncoderPos(), setpoint);

    double maxPower = 0.25;
    if(getEncoderPos() > 31.0) maxPower = 0.7;
    if(Math.abs(output) > maxPower) output = maxPower * Math.signum(output);

    motor.set(output);
  }
}

