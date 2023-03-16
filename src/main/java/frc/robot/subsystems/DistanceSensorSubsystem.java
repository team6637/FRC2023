// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DistanceSensorSubsystem extends SubsystemBase {
  
  private final Rev2mDistanceSensor distanceSensor;

  public DistanceSensorSubsystem() {

    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    distanceSensor.setAutomaticMode(true);

  }

  public double getSensorValue() {
    return distanceSensor.getRange();
  }

  @Override
  public void periodic() {
    if(Constants.DistanceSensorConstants.isTunable) {
      SmartDashboard.putNumber("sensor range", getSensorValue());
    }
  }
}
