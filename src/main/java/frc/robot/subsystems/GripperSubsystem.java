// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

  CANSparkMax motor1 = new CANSparkMax(Constants.GripperConstants.GRIPPER_MOTOR_PORT, MotorType.kBrushless);
  RelativeEncoder motor1Encoder = motor1.getEncoder();

  public GripperSubsystem() {}

  @Override
  public void periodic() {
  }

  public void open() {
    motor1.set(0.2);
  }
  public void close() {
    motor1.set(-0.2);
  }
  public void stop() {
    motor1.set(0);
  }

}
