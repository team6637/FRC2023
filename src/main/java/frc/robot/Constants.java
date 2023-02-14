// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.util.COTSFalconSwerveConstants;


public final class Constants {

    // ARM 
    public static class ArmConstants {
        public static final int LEFT_ARM_PORT = 51;
        public static final int RIGHT_ARM_PORT = 39;
        public static final int ENCODER_PORT = 0;

        public static final boolean isTunable = true;

        public static final double minAngle = -62.0;
        public static final double maxAngle = 40.0;
    }

    // EXTENDER
    public static class ExtenderConstants {
        public static final int EXTENDER_PORT = 40;
        public static final double EXTENDER_MIN_EXTENSION = 0.0;
        public static final double EXTENDER_MAX_EXTENSION = 155.0; 

        public static final boolean isTunable = true;

    }

    // GRIPPER
    public static class GripperConstants {
        public static final int GRIPPER_MOTOR_PORT = 34;

        public static final boolean isTunable = true;

        public static final double fullOpen = 5.0;
        public static final double fullClosed = 50.0;

        public static final double closeCone = 48.0;
        public static final double closeCube = 30.0;
    }

    public static class LimeLightConstants {
        public static final double limelightMaxSpeed = 0.3;
    }

    // SWERVE
    public static class Swerve {
        public static final double maxTurnSpeed = 0.5;
    
        public static final double trackWidth = Units.inchesToMeters(23.5);
        public static final double wheelBase = Units.inchesToMeters(23.5);
        public static final double wheelDiameter = 0.1016;
        public static final double wheelCircumference = wheelDiameter * Math.PI;
    
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
         /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.0;

        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    
    
        public static final double maxAccelerationMetersPerSecond = 9.0;
        public static final double maxAccelerationRadiansPerSecond = Math.PI;
    
    
    
        public static final COTSFalconSwerveConstants chosenModule =
          COTSFalconSwerveConstants.SDSMK3(COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Standard);
    
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;
    
    
    
        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;
    
        /* Drive Motor PID Values */
        public static final double turnKP = 0.6;
        public static final double turnKI = 0.0;
        public static final double turnKD = 0.0;
        public static final double turnKF = 0.0;
    
        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);
    
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
        public static final IdleMode turnNeutralMode = IdleMode.kCoast;

        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    
        //public static final double turnConversionFactorAngToRad = Rotation2d.fromDegrees(360.0 / chosenModule.angleGearRatio).getRadians();
        public static final double turnConversionFactorAngToRad = 1 / chosenModule.angleGearRatio * 2 * Math.PI;
    
        public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxSpeed, maxAccelerationMetersPerSecond).setKinematics(swerveKinematics);

        public static PathConstraints pathPlannerConstraints = new PathConstraints(maxSpeed, maxAccelerationMetersPerSecond);

        public static final TrapezoidProfile.Constraints zConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAccelerationRadiansPerSecond);
    
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 2;
          public static final int canCoderID = 21;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(264.9);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 22;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221.2);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 23;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(99.2);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 24;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.0);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
