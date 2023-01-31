// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixySubsystem extends SubsystemBase {

	private final Pixy2 pixy;
	private double x;
	private int w;
	private boolean targetIsSeen = false;

	public PixySubsystem() {
		x = 0;
		pixy = Pixy2.createInstance(new SPILink());
		pixy.init();
		this.w = pixy.getFrameWidth();
	}

	public void activate(String color) {
		pixy.setLamp((byte) 1, (byte) 1);
    	pixy.setLED(255, 255, 255);

		if(color == "red") {
			pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		} else {
			pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG2, 25);
		}
	
		ArrayList<Block> blocks = pixy.getCCC().getBlockCache();
	
		if (blocks.size() <= 0) {
		  	this.targetIsSeen = false;
			this.x = 0;
			return;
		}
	
		Block largestBlock = null;
		for (Block block : blocks) {
		  if (largestBlock == null) {
			largestBlock = block;
		  } else if (block.getWidth() > largestBlock.getWidth()) {
			largestBlock = block;
		  }
		}
		this.x = (largestBlock.getX() - w/2) * -0.02;
		this.targetIsSeen = true;
	}

	public void deactivate() {
		pixy.setLamp((byte) 0, (byte) 0);
		this.targetIsSeen = false;
		this.x = 0;
	}

	public double getOffset() {
		return this.x;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("pixy x", getOffset());
		SmartDashboard.putBoolean("pixy sees target", this.targetIsSeen);
	}
}