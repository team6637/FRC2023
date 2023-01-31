// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
	
	private final NetworkTable table;

	public LimelightSubsystem() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		setPipeline(0);
		setupDriveMode();
	}

	public boolean isTarget() {
		return getValue("tv").getDouble(0.0) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.0);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.0);
	}

	public void setLedMode(int ledMode) {
		getValue("ledMode").setNumber(ledMode);
  	}
  
	public void setCameraMode(int cameraMode) {
		getValue("camMode").setNumber(cameraMode);
 	}

	public void setStream(int streamMode) {
		getValue("stream").setNumber(streamMode);
 	}
	
	public void setupAutoAim() {
		setLedMode(3);
		setCameraMode(0);
	}
	  
	public void setupDriveMode() {
		setLedMode(1);
		setCameraMode(0);
		setStream(2);
	}

	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

  	private NetworkTableEntry getValue(String key) {
		return table.getEntry(key);
	}

	// return inches
	public double getDistance() {
		return (104.0-25.0) / Math.tan(Math.toRadians(34.74 + getTx()));
	}
}