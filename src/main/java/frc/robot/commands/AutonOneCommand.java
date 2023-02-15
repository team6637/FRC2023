// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class AutonOneCommand extends SequentialCommandGroup {
    public AutonOneCommand(Swerve swerve, ArmSubsystem arm, ExtenderSubsystem extender, GripperSubsystem gripper) {
        
        // DRIVE FORWARD (1 FOOT)
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(2.17, 4.44, new Rotation2d(Math.toRadians(180.0))), 
                new Pose2d(1.87, 4.44, new Rotation2d(Math.toRadians(180.0)))
            ),
            Constants.Swerve.trajectoryConfig
        );
        SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
            trajectory,
            swerve::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.autonXKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonYKp, 0.0, 0.0),
            new ProfiledPIDController(Constants.Swerve.autonXKp, 0, 0, Constants.Swerve.zConstraints),
            swerve::setModuleStates,
            swerve
        );

        // DRIVE BACKWARD (1 FOOT)
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(1.87, 4.44, new Rotation2d(Math.toRadians(180.0))),
                new Pose2d(2.17, 4.44, new Rotation2d(Math.toRadians(180.0)))
            ),
            Constants.Swerve.trajectoryConfig
        );
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
            trajectory,
            swerve::getPose, // Functional interface to feed supplier
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.autonXKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonYKp, 0.0, 0.0),
            new ProfiledPIDController(Constants.Swerve.autonXKp, 0, 0, Constants.Swerve.zConstraints),
            swerve::setModuleStates,
            swerve
        );

        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("auton1", Constants.Swerve.maxSpeed, Constants.Swerve.maxAccelerationMetersPerSecond);

        PIDController autonXController = new PIDController(Constants.Swerve.autonXKp, 0, 0);
        PIDController autonYController = new PIDController(Constants.Swerve.autonYKp, 0, 0);
        PIDController autonZController = new PIDController(Constants.Swerve.autonZKp, 0, 0);

        PPSwerveControllerCommand swerveControllerCommand3 = new PPSwerveControllerCommand(
            trajectory3,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            autonXController,
            autonYController,
            autonZController,
            swerve::setModuleStates,
            swerve
        );
       
        addCommands(

            // DRIVE FORWARD AND DROP
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.closeCube)),
            new InstantCommand(() ->  arm.setSetpoint(14.0)),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> extender.setSetpoint(120)),
            new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand1,
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.openGripper()),
            new WaitCommand(1.0),

            // DRIVE BACK AND RESET
            swerveControllerCommand2,
            new InstantCommand(() ->  extender.setSetpoint(0)),
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.setSetpoint(20)),
            new InstantCommand(() -> arm.setSetpoint(Constants.ArmConstants.minAngle)),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.fullOpen))




            //new InstantCommand(() -> swerve.resetOdometry(trajectory3.getInitialPose()))
            //swerveControllerCommand3
        );
    }

}
