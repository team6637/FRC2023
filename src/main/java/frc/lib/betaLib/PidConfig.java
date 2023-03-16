// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.betaLib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PidConfig {
    private String key;
    private Double kP;
    private Double kI;
    private Double kD;
    private Double maxVelocity;
    private Double maxAcceleration;
    private boolean isTunable;

    public PidConfig(String key, double kP, boolean isTunable) {
        this.key = key;
        this.kP = kP;
        this.isTunable = isTunable;

        if(isTunable) {
            SmartDashboard.putNumber(key + " kp", kP);
        }
    }

    public PidConfig(String key, double kP, double kI, double kD,  boolean isTunable) {
        this.key = key;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.isTunable = isTunable;

        if(isTunable) {
            SmartDashboard.putNumber(key + " kp", kP);
            SmartDashboard.putNumber(key + " ki", kI);
            SmartDashboard.putNumber(key + " kd", kD);
        }
    }

    public PidConfig(String key, double kP, double kI, double kD, double maxVelocity, double maxAcceleration, boolean isTunable) {
        this.key = key;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.isTunable = isTunable;

        if(this.isTunable) {
            SmartDashboard.putNumber(key + " kp", kP);
            SmartDashboard.putNumber(key + " ki", kI);
            SmartDashboard.putNumber(key + " kd", kD);
            SmartDashboard.putNumber(key + " maxVelocity", maxVelocity);
            SmartDashboard.putNumber(key + " maxAcceleration", maxAcceleration);
        }
    }

    public void updateFromSmartDashboard() {
        if(this.kP != null) {
            this.kP = SmartDashboard.getNumber(key + " kp", kP);
        }

        if(this.kI != null) {
            this.kI = SmartDashboard.getNumber(key + " kp", kI);
        }

        if(this.kD != null) {
            this.kD = SmartDashboard.getNumber(key + " kp", kD);
        }

        if(this.maxVelocity != null) {
            this.maxVelocity = SmartDashboard.getNumber(key + " kp", maxVelocity);
        }

        if(this.maxAcceleration != null) {
            this.maxAcceleration = SmartDashboard.getNumber(key + " kp", maxAcceleration);
        }
    }

    public void setKp(double v) {
        this.kP = v;
    }

    public void setKI(double v) {
        this.kI = v;
    }

    public void setKD(double v) {
        this.kD = v;
    }

    public void setMaxVelocity(double v) {
        this.maxVelocity = v;
    }

    public void setMaxAcceleration(double v) {
        this.maxAcceleration = v;
    }

    public double getKp() {
        if(this.kP == null) {
            return 0.0;
        }
        return this.kP;
    }

    public double getKi() {
        if(this.kI == null) {
            return 0.0;
        }
        return this.kI;
    }

    public double getKd() {
        if(this.kD == null) {
            return 0.0;
        }
        return this.kD;
    }

    public double getMaxVelocity() {
        return this.maxVelocity;
    }

    public double getMaxAcceleration() {
        return this.maxAcceleration;
    }
    
}
