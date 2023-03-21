// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final double minSetpoint = Constants.ExtenderConstants.EXTENDER_MIN_EXTENSION;
  private final double maxSetpoint = Constants.ExtenderConstants.EXTENDER_MAX_EXTENSION;
  private double setpoint = minSetpoint;
  private double kp = 0.07;
  private boolean usingPID = true;
  private double setpointIncrementer = 1.0;
  private final PidConfig PidConfig = new PidConfig("extender", kp, Constants.ExtenderConstants.isTunable);
  private final PIDController pid = new PIDController(PidConfig.getKp(), PidConfig.getKi(), PidConfig.getKd());

  public ExtenderSubsystem() {

    pid.setTolerance(20.0);

    motor = new CANSparkMax(ExtenderConstants.EXTENDER_PORT, MotorType.kBrushless);
    motor.setInverted(true);

    encoder = motor.getEncoder();

    setSetpoint(minSetpoint);

    if (Constants.ExtenderConstants.isTunable) {
      SmartDashboard.putNumber("extender setpoint", setpoint);
    }
  }

  public void extend() {
    if (usingPID) {
      setSetpoint(setpoint += setpointIncrementer);
    } else {
      motor.set(1.0);
    }
  }

  public void retract() {
    if (usingPID) {
      setSetpoint(setpoint -= setpointIncrementer);
    } else {
      motor.set(-1.0);
    }
  }

  public void stop() {
    if (!usingPID) {
      motor.set(0);
    }
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void setSetpoint(double newSetpoint) {
    if (newSetpoint > maxSetpoint) {
      setpoint = maxSetpoint;
    } else if (newSetpoint < minSetpoint) {
      setpoint = minSetpoint;
    } else {
      setpoint = newSetpoint;
    }
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    if (Constants.ExtenderConstants.isTunable) {
      SmartDashboard.putNumber("extender encoder", getPosition());
    }

    double speed = pid.calculate(getPosition(), setpoint);
    if (usingPID) {
      motor.set(speed);
    }

  }
}
