// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSubsystem extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(ExtenderConstants.LEFT_EXTENDER_PORT, MotorType.kBrushless);

  private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT);

  final int maxLength = 10000;
  final int leastLength = 0;

  public ExtenderSubsystem() {}

  public void extend() {
    if(encoder.get() <= maxLength) {
      motor.set(0.2);
    } else {
      motor.set(0);
    }
  }
  public void retract() {
    if(encoder.get() <= maxLength) {
      motor.set(-0.2);
    } else {
      motor.set(0);
    }
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
