package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class TurnToAngleCommand extends CommandBase {

    private final Swerve swerve;
    private boolean complete = false;
    private double angle;
    private Timer timer = new Timer();
    private double timeout;
    public TurnToAngleCommand(Swerve subsystem, double degrees, double timeoutS){
        swerve = subsystem;
        angle = degrees;
        timeout = timeoutS;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        complete = false;
    }
    
    @Override
    public void execute(){
        double gyroAngle = swerve.getHeading();

        final double kP = 0.2;
        SmartDashboard.putNumber("gyroAngle", gyroAngle);
    
        if (angle > 180) {
            angle = -(360 - angle);
        } else if (angle < -180) {
            angle = 360 + angle;
        }
    
        double err = angle - gyroAngle;
        double speed = MathUtil.clamp(err * kP, -Constants.Swerve.maxAngularVelocity*0.5, Constants.Swerve.maxAngularVelocity*0.5);
    
        if (Math.abs(err) > 2 && timer.get() < timeout) {

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, speed);
            SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerve.setModuleStates(moduleStates);
            
        } else {
            complete = true;
        }
    }

    @Override
    public void end(boolean inturrupted){
        swerve.stopModules();
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return complete;
    }
}