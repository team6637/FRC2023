// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private SwerveModule[] mSwerveMods;

  // private final SwerveModule frontLeft = new SwerveModule(
  //   0,
  //   Constants.Swerve.Mod0.driveMotorID,
  //   Constants.Swerve.Mod0.angleMotorID,
  //   Constants.Swerve.Mod0.canCoderID,
  //   Constants.Swerve.Mod0.angleOffset.getRadians()
  // );

  // private final SwerveModule frontRight = new SwerveModule(
  //   1,
  //   Constants.Swerve.Mod1.driveMotorID,
  //   Constants.Swerve.Mod1.angleMotorID,
  //   Constants.Swerve.Mod1.canCoderID,
  //   Constants.Swerve.Mod1.angleOffset.getRadians()
  // );

  // private final SwerveModule backLeft = new SwerveModule(
  //   2,
  //   Constants.Swerve.Mod2.driveMotorID,
  //   Constants.Swerve.Mod2.angleMotorID,
  //   Constants.Swerve.Mod2.canCoderID,
  //   Constants.Swerve.Mod2.angleOffset.getRadians()
  // );

  // private final SwerveModule backRight = new SwerveModule(
  //   3,
  //   Constants.Swerve.Mod3.driveMotorID,
  //   Constants.Swerve.Mod3.angleMotorID,
  //   Constants.Swerve.Mod3.canCoderID,
  //   Constants.Swerve.Mod3.angleOffset.getRadians()
  // );

  // GYRO
  private final Pigeon2 gyro;

  private Field2d field;
  
  // ODOMETER
  private SwerveDriveOdometry odometry;

  public Swerve() {
    gyro = new Pigeon2(9, "Gary");
    zeroGyroscope();

    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroscopeRotation(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

  }


  public void zeroGyroscope() {
      gyro.setYaw(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public double getHeading() {
    return Math.IEEEremainder(getGyroscopeRotation().getDegrees(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("swerve heading", getHeading());

    odometry.update(getGyroscopeRotation(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAbsoluteEncoderDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

}
