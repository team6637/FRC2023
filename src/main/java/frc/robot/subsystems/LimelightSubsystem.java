// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
	
	private final NetworkTable table;
	private final Servo servo;
	private double kp = 0.05;
	private boolean visionActive = false;
	private final double servoUpPosition = 0.72;
	private final double servoDownPosition = 0.53;

	public LimelightSubsystem() {
		table = NetworkTableInstance.getDefault().getTable("limelight");

		servo = new Servo(0);
		servo.set(servoUpPosition);

		setVisionMode("off");

		SmartDashboard.putNumber("ll kp", kp);
		setStream(2);

	}

	public void servoUp() {
		servo.set(servoUpPosition);
	}
	public void servoDown() {
		servo.set(servoDownPosition);
	}

	public boolean isTarget() {
		return getValue("tv").getDouble(0.0) == 1;
	}
	public double getRawTx() {
		return getValue("tx").getDouble(0.0);
	}
	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		double output = getValue("tx").getDouble(0.0) * kp;
		if (Math.abs(output) >= Constants.LimeLightConstants.limelightMaxSpeed) {
			output = Constants.LimeLightConstants.limelightMaxSpeed * Math.signum(output);
		}
		if (!visionActive) {
			output = 0.0;
		}
		return -output;
	}

	public Boolean atTxTarget() {
		return this.getTx() < 1;
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
	
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

  	private NetworkTableEntry getValue(String key) {
		return table.getEntry(key);
	}

	public void setVisionMode(String visionMode) {
		setLedMode(0);
		setCameraMode(0);
		if (visionMode == "cube") { //april
			servoUp();
			setPipeline(0);
			visionActive = true;
		} else if (visionMode == "object") {
			servoDown();
			setPipeline(1);
			visionActive = true;
		} else if (visionMode == "cone") { //reflective
			servoUp();
			setPipeline(2);
			visionActive = true;
		} else {
			visionActive = false;
			setLedMode(1);
			setCameraMode(1);
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("ll get tx", getTx());
	}
}