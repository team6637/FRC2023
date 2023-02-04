// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(Constants.GripperConstants.GRIPPER_MOTOR_PORT, MotorType.kBrushless);
  DutyCycleEncoder encoder1 = new DutyCycleEncoder(1);
  double angleOffset = 0;

  public GripperSubsystem() {}

  public void open() {
    motor1.set(0.2);
  }
  public void close() {
    motor1.set(-0.2);
  }
  public void stop() {
    motor1.set(0);
  }
  public double getDegrees() {
    return encoder1.get()*360-angleOffset;
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Throughbore encoder value", getDegrees());
  }
}

