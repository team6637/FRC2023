// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.lib.config.CTREConfigs;
import frc.lib.math.Conversions;


public class SwerveModule {

    private final TalonFX driveMotor;

    private final CANSparkMax turnMotor;
    private final PIDController turnPidController;

    private RelativeEncoder integratedAngleEncoder;

    private final CANCoder absoluteEncoder;
    public final double absoluteEncoderOffsetRad;

    public final int swervePodId;

    public Rotation2d lastAngle;

    public SwerveModule(
            int swervePodId,
            int driveMotorId, 
            int turnMotorId,
            int absoluteEncoderId,
            double absoluteEncoderOffsetRad
        ) {
        
        this.swervePodId = swervePodId;

        driveMotor = new TalonFX(driveMotorId, "Gary");
        Robot.ctreConfigs = new CTREConfigs();
        configDriveMotor();

        turnMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);
        integratedAngleEncoder = turnMotor.getEncoder();
        configTurnMotor();

        turnPidController = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new CANCoder(absoluteEncoderId, "Gary");
        configAbsoluteEncoder();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        resetEncoders();
        lastAngle = getState().angle;
    }
    
    public void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }
        

    public void configTurnMotor() {
        turnMotor.restoreFactoryDefaults();
        //CANSparkMaxUtil.setCANSparkMaxBusUsage(turnMotor, Usage.kPositionOnly);
        //turnMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        turnMotor.setInverted(false);
        turnMotor.setIdleMode(IdleMode.kCoast);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.turnConversionFactorAngToRad);
        turnMotor.burnFlash();
        Timer.delay(1);
    }

    public void configAbsoluteEncoder() {
        absoluteEncoder.configFactoryDefault();
        //CANCoderUtil.setCANCoderBusUsage(absoluteEncoder, CCUsage.kMinimal);
        absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getDrivePositionMeters() {
        return Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    }

    public double getTurnPosition() {
        return integratedAngleEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurnVelocity() {
        return integratedAngleEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRadians() {
        Rotation2d v = Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
        double radians = v.getRadians() - absoluteEncoderOffsetRad;
        return radians;
    }

    public double getAbsoluteEncoderDegrees() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        integratedAngleEncoder.setPosition(getAbsoluteEncoderRadians());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    } 

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);

        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : state.angle;


        double percentOutput = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);

        double turnOutput = turnPidController.calculate(getTurnPosition(), angle.getRadians());

        SmartDashboard.putNumber("swerve " + swervePodId + " turn output", turnOutput);
        SmartDashboard.putNumber("swerve " + swervePodId + " turn position", getTurnPosition());
        SmartDashboard.putNumber("swerve " + swervePodId + " turn setpoint", state.angle.getRadians());
        turnMotor.set(turnOutput);

        lastAngle = angle;

        SmartDashboard.putString("swerve[" + swervePodId + "] state", state.toString());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio),
            new Rotation2d(getTurnPosition())
        );
      }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(0);
    }

}