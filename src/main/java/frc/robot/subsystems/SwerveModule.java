// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.lib.config.CTREConfigs;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.Conversions;
import frc.lib.math.OnboardModuleState;


public class SwerveModule {

    private final TalonFX driveMotor;

    private final CANSparkMax turnMotor;
    private final PIDController turnPidController;

    private RelativeEncoder integratedAngleEncoder;

    private final CANCoder absoluteEncoder;
    public double absoluteEncoderOffsetRad;

    public final int moduleNumber;

    public Rotation2d lastAngle;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(
            int moduleNumber,
            SwerveModuleConstants constants
        ) {
        
        this.moduleNumber = moduleNumber;

        driveMotor = new TalonFX(constants.driveMotorID, "Gary");
        Robot.ctreConfigs = new CTREConfigs();
        configDriveMotor();

        turnMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = turnMotor.getEncoder();
        configTurnMotor();

        turnPidController = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        absoluteEncoder = new CANCoder(constants.cancoderID, "Gary");
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

    public void setDesiredState(SwerveModuleState desiredState, Boolean isOpenLoop) {

        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        //state = SwerveModuleState.optimize(state, getState().angle);

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;


        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);

        // if(isOpenLoop){
        //     double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        //     driveMotor.set(ControlMode.PercentOutput, percentOutput);
        // }
        // else {
        //     double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        //     driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        // }

        double turnOutput = turnPidController.calculate(getTurnPosition(), angle.getRadians());

        turnMotor.set(turnOutput);
        lastAngle = angle;
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
