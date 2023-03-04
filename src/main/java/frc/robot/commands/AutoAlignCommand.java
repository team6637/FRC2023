package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoAlignCommand extends CommandBase {
  
  private LimelightSubsystem vision;
  private Swerve swerve;
  private final double kp = 0.1;
  private final double kd = 0.02;
  double output = 0.0;
  PIDController PID = new PIDController(kp, 0, kd);
  int counter = 0;

  public AutoAlignCommand(LimelightSubsystem vision, Swerve swerve) {
    this.swerve = swerve;
    this.vision = vision;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    vision.setVisionMode("object");
    PID.setTolerance(2.5, 3);
    SmartDashboard.putNumber("auto align command state", 0.0);
  }

  @Override
  public void execute() {
    counter++;
    SmartDashboard.putNumber("auto align command state", 1.0);


    output = PID.calculate(vision.getRawTx(), 0.0);
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
    SmartDashboard.putNumber("auto align command state", 2.0);

}

  @Override
  public boolean isFinished() {
    return PID.atSetpoint()  && vision.isTarget() && counter > 50;
  }
}