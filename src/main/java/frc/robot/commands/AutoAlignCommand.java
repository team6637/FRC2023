package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.betaLib.PidConfig;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoAlignCommand extends CommandBase {
  
  private LimelightSubsystem vision;
  private Swerve swerve;

  private final double kp = 0.1;
  private final double kd = 0.02;
  double output = 0.0;
  int counter = 0;

  private final PidConfig pidConfig = new PidConfig("AutoAlign", kp, 0, kd, Constants.AutonConstants.AutoAlignIsTunable);
  private final PIDController pid = new PIDController(pidConfig.getKp(), pidConfig.getKi(), pidConfig.getKd());

  public AutoAlignCommand(LimelightSubsystem vision, Swerve swerve) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    vision.setVisionMode("object");
    pid.setTolerance(2.5, 3);
    if (Constants.AutonConstants.AutoAlignIsTunable) {
      SmartDashboard.putNumber("auto align command state", 0.0);
    }
  }

  @Override
  public void execute() {
    counter++;
    if (Constants.AutonConstants.AutoAlignIsTunable) {
      SmartDashboard.putNumber("auto align command state", 1.0);

      pidConfig.updateFromSmartDashboard();
    }

    output = pid.calculate(vision.getRawTx(), 0.0);
    if (Math.abs(output)>=0.6) {
      output = 0.6*Math.signum(output);
    }

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, output, 0.0);
    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerve.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    vision.setVisionMode("off"); 
    if (Constants.AutonConstants.AutoAlignIsTunable) {
      SmartDashboard.putNumber("auto align command state", 2.0);
    }
  }

  @Override
  public boolean isFinished() {
    return pid.atSetpoint()  && vision.isTarget() && counter > 50;
  }
}