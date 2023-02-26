// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class AutonOneCommand extends SequentialCommandGroup {
    public AutonOneCommand(Swerve swerve, ArmSubsystem arm, ExtenderSubsystem extender, GripperSubsystem gripper, LimelightSubsystem vision, Boolean isAuton3) {

        double initialXPosition = isAuton3 ? 1.87 : 1.87;
        double initialYPosition = isAuton3 ? 1.05 : 4.44;
        
        // DRIVE TO OBJECT
        String path1 = isAuton3 ? "auton3" : "auton1";
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath(path1, Constants.Swerve.maxSpeed, Constants.Swerve.maxAccelerationMetersPerSecond);

        PPSwerveControllerCommand swerveControllerCommand1 = new PPSwerveControllerCommand(
            trajectory1,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.autonXKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonYKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonZKp, 0, 0),
            swerve::setModuleStates,
            true,
            swerve
        );

        String path2 = isAuton3 ? "auton3back" : "auton1back";
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath(path2, Constants.Swerve.maxSpeed, Constants.Swerve.maxAccelerationMetersPerSecond);

        PPSwerveControllerCommand swerveControllerCommand2 = new PPSwerveControllerCommand(
            trajectory2,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.Swerve.autonXKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonYKp, 0.0, 0.0),
            new PIDController(Constants.Swerve.autonZKp, 0, 0),
            swerve::setModuleStates,
            true,
            swerve
        );
       
        addCommands(

            // SET ROBOT POSE
            new InstantCommand(() -> swerve.resetOdometry(new Pose2d(initialXPosition, initialYPosition, new Rotation2d(Math.toRadians(180.0))))),

            // LIFT ARM
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.closeCube)),
            new InstantCommand(() ->  arm.setSetpoint(14.0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> extender.setSetpoint(120)),
            
            // DRIVE FORWARD 1 FOOT
            new DriveStraightCommand(swerve, -0.1, true),

            // DROP OBJECT
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.autonSetSetpoint(Constants.GripperConstants.fullOpenWhenExtended)),
            new WaitCommand(0.5),

            // DRIVE BACK 1 FOOT
            new DriveStraightCommand(swerve, 0.3, true),

            new TurnCommand(swerve, 0.0),

            // DRIVE TO OBJECT
            new ParallelRaceGroup(
                swerveControllerCommand1,
                new SequentialCommandGroup(
                    // RESET ARM
                    new InstantCommand(() ->  extender.setSetpoint(0)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> extender.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(20)),
                    new InstantCommand(() -> arm.setSetpoint(Constants.ArmConstants.minAngle)),
                    new WaitCommand(0.1),
                    new WaitUntilCommand(() -> arm.atSetpoint()),
                    new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.fullOpen))
                )
            ),

            // VISION CORRECTION
            new InstantCommand(()->vision.setVisionMode("object"), vision),
            new SwerveTeleopCommand(swerve, ()->0.0, ()->0.0, ()->0.0, ()->vision.getTx(), ()->1.0, true),
            new InstantCommand(()->vision.setVisionMode("off"), vision),

            // Lift Arm
            new InstantCommand(() ->  arm.setSetpoint(-55.0)),

            // Drive Forward
            new DriveStraightCommand(swerve, 0.78, false),

            // CLOSE GRIPPER
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.closeCone)),
            new WaitCommand(0.8),
            new InstantCommand(() ->  arm.setSetpoint(-30.0)),

            // FOLLOW PATH BACK
            swerveControllerCommand2,

            // TURN AROUND
           // new TurnCommand(swerve, 180.0),

            // VISION ADJUSTMENT
            // new InstantCommand(()-> vision.setVisionMode("cone"), vision),
            // new SwerveTeleopCommand(swerve, ()->0.0, ()->0.0, ()->0.0, ()->vision.getTx(), ()->1.0, true),
            // new InstantCommand(()-> vision.setVisionMode("off"), vision),

            // PLACE CONE
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.closeCone)),
            new InstantCommand(() ->  arm.setSetpoint(30.0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> extender.setSetpoint(155)),
            
            // DRIVE FORWARD 1 FOOT
            new DriveStraightCommand(swerve, -0.3, true),

            // DROP OBJECT
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.autonSetSetpoint(Constants.GripperConstants.fullOpenWhenExtended)),
            new WaitCommand(0.1),

            // RESET ARM
            new InstantCommand(() ->  extender.setSetpoint(0)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> extender.atSetpoint()),
            new InstantCommand(() -> gripper.setSetpoint(20)),
            new InstantCommand(() -> arm.setSetpoint(Constants.ArmConstants.minAngle)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> arm.atSetpoint()),
            new InstantCommand(() -> gripper.setSetpoint(Constants.GripperConstants.fullOpen))
        );
    }
}