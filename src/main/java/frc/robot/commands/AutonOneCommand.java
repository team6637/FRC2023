// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class AutonOneCommand extends SequentialCommandGroup {
    public AutonOneCommand(Swerve m_driveSubsystem) {
        
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("test", Constants.Swerve.maxSpeed, Constants.Swerve.maxAccelerationMetersPerSecond);

        PIDController autonXController = new PIDController(1.0, 0, 0);
        PIDController autonYController = new PIDController(1.0, 0, 0);
        ProfiledPIDController autonZController = new ProfiledPIDController(1.0, 0, 0, Constants.Swerve.zConstraints);
        autonZController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
            trajectory,
            m_driveSubsystem::getPose,
            Constants.Swerve.swerveKinematics,
            autonXController,
            autonYController,
            autonZController,
            m_driveSubsystem::setModuleStates,
            m_driveSubsystem
        );
       
        addCommands(
            new InstantCommand(() -> m_driveSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }

}
