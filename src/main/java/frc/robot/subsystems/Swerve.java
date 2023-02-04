// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
    0,
    Constants.Swerve.Mod0.driveMotorID,
    Constants.Swerve.Mod0.angleMotorID,
    Constants.Swerve.Mod0.canCoderID,
    Constants.Swerve.Mod0.angleOffset.getRadians()
  );

  private final SwerveModule frontRight = new SwerveModule(
    1,
    Constants.Swerve.Mod1.driveMotorID,
    Constants.Swerve.Mod1.angleMotorID,
    Constants.Swerve.Mod1.canCoderID,
    Constants.Swerve.Mod1.angleOffset.getRadians()
  );

  private final SwerveModule backLeft = new SwerveModule(
    2,
    Constants.Swerve.Mod2.driveMotorID,
    Constants.Swerve.Mod2.angleMotorID,
    Constants.Swerve.Mod2.canCoderID,
    Constants.Swerve.Mod2.angleOffset.getRadians()
  );

  private final SwerveModule backRight = new SwerveModule(
    3,
    Constants.Swerve.Mod3.driveMotorID,
    Constants.Swerve.Mod3.angleMotorID,
    Constants.Swerve.Mod3.canCoderID,
    Constants.Swerve.Mod3.angleOffset.getRadians()
  );

  // GYRO
  private final Pigeon2 m_pigeon = new Pigeon2(9, "Gary");

  public Swerve() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroGyroscope();
      } catch (Exception e) { 

      }
    }).start();
  }


  public void zeroGyroscope() {
      m_pigeon.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public double getHeading() {
    return Math.IEEEremainder(getGyroscopeRotation().getDegrees(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleState(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("swerve heading", getHeading());
    SmartDashboard.putNumber("fl abs angle", frontLeft.getAbsoluteEncoderDegrees());
    SmartDashboard.putNumber("fr abs angle", frontRight.getAbsoluteEncoderDegrees());
    SmartDashboard.putNumber("fr adjusted angle", frontRight.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("bl abs angle", backLeft.getAbsoluteEncoderDegrees());
    SmartDashboard.putNumber("bl adjusted angle", backLeft.getAbsoluteEncoderRadians());
    SmartDashboard.putNumber("br abs angle", backRight.getAbsoluteEncoderDegrees());
  }

}
