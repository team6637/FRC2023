// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;


public final class Constants {
    public static class DriveConstants {
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.5);

      // reduce to slow robot
      public static final double MAX_VOLTAGE = 12.0;

      public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.5);
      public static final int DRIVETRAIN_PIGEON_ID = 9;

      public static final double AUTON_TURN_KP = 14.0;
      public static final double AUTON_X_KP = 2.0;
      public static final double AUTON_Y_KP = 2.0;

      // least amount of power required to overcome the drivetrain's static friction
      public static final double DRIVETRAIN_KS = 0.2;
      
      public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
      
      public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
      public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21;
      public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(83.23);

      public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
      public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
      public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; 
      public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(41.13);

      public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; 
      public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
      public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; 
      public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(279.32);

      public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
      public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
      public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24; 
      public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(322.47); 

  }

  public static class ArmConstants {
    public static final int LEFT_ARM_PORT = 51;
    public static final int RIGHT_ARM_PORT = 39;
    public static final int ENCODER_PORT = 0;

  }

  public static class ExtenderConstants {
    public static final int LEFT_EXTENDER_PORT = 32;
    public static final int RIGHT_EXTENDER_PORT = 33;

  }

  public static class GripperConstants {
    public static final int GRIPPER_MOTOR_PORT = 34;
  }
}
