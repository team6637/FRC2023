// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // 3.87119 m/s
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI
        / 1.074;

    public double get_max_velocity() {
        return MAX_VELOCITY_METERS_PER_SECOND;
    }

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public double get_angular_velocity() {
        return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // SWERVE MODULES
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    // CONSTRUCTOR
    public DriveSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        this.m_frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
                .withDriveMotor(MotorType.FALCON, DriveConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR, "Gary")
                .withSteerMotor(MotorType.NEO, DriveConstants.FRONT_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(DriveConstants.FRONT_LEFT_MODULE_STEER_ENCODER, "Gary")
                .withSteerOffset(DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();

        this.m_frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
            .withDriveMotor(MotorType.FALCON, DriveConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, "Gary")
            .withSteerMotor(MotorType.NEO, DriveConstants.FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(DriveConstants.FRONT_RIGHT_MODULE_STEER_ENCODER, "Gary")
            .withSteerOffset(DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET)
            .build();

        this.m_backLeftModule = new MkSwerveModuleBuilder()
             .withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
            .withDriveMotor(MotorType.FALCON, DriveConstants.BACK_LEFT_MODULE_DRIVE_MOTOR, "Gary")
            .withSteerMotor(MotorType.NEO, DriveConstants.BACK_LEFT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(DriveConstants.BACK_LEFT_MODULE_STEER_ENCODER, "Gary")
            .withSteerOffset(DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET)
            .build();

        this.m_backRightModule = new MkSwerveModuleBuilder()
            .withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK3_STANDARD)
            .withDriveMotor(MotorType.FALCON, DriveConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, "Gary")
            .withSteerMotor(MotorType.NEO, DriveConstants.BACK_RIGHT_MODULE_STEER_MOTOR)
            .withSteerEncoderPort(DriveConstants.BACK_RIGHT_MODULE_STEER_ENCODER, "Gary")
            .withSteerOffset(DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET)
            .build();

            m_odometry = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), getPositions());
    }

    // GYRO
    private final Pigeon2 m_pigeon = new Pigeon2(9, "Gary");

    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    // KINEMATICS!
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // ODOMETRY
    private SwerveDriveOdometry m_odometry;

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(this.getGyroscopeRotation(), getPositions(), pose);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    // CHASSIS SPEEDS
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public ChassisSpeeds getSpeeds() {
        return m_chassisSpeeds;
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(),
            m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(),
            m_backRightModule.getPosition()
        };
    }

    public void setModuleStates(SwerveModuleState[] states) {
        //SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * DriveConstants.MAX_VOLTAGE, states[0].angle.getRadians());

        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * DriveConstants.MAX_VOLTAGE, states[1].angle.getRadians());

        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * DriveConstants.MAX_VOLTAGE, states[2].angle.getRadians());

        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * DriveConstants.MAX_VOLTAGE, states[3].angle.getRadians());        
    }

    // DRIVE - CALLED FROM COMMANDS
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        // put pose info in SmartDashboard
        SmartDashboard.putNumber("pose x", getPose().getX());
        SmartDashboard.putNumber("pose y", getPose().getY());
        SmartDashboard.putNumber("pose rotation", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("drive gyro rotation", getGyroscopeRotation().getDegrees());
        m_odometry.update(
            this.getGyroscopeRotation(),
            getPositions()
        );

        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        setModuleStates(states);

    }
}
