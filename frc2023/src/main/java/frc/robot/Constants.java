// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModule;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SwerveModuleConstants{
    public static final double driveKS = 0.71003;
    public static final double driveKV = 2.2783;
    public static final double driveKA = 0.25953;

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS,driveKV,driveKA);
    public static final double TURNING_KP = .25;

    public static final double CLOSED_LOOP_RAMP_RATE  = .2;
    public static final double OPEN_LOOP_RAMP_RATE  = .25;

    public static final double TURN_GEAR_RATIO  = 12.8;
    public static final double DRIVE_GEAR_RATIO  = 6.75;

    public static final String SWERVE_CANIVORE_ID = "swerve";

    public static final int TIMEOUT_MS = 50;
}

  public static class DriveConstants{
    public static final int MAX_TANGENTIAL_VELOCITY = 4; 
    public static final double MAX_TELE_TANGENTIAL_VELOCITY = 3.5; 
    public static final double MAX_TELE_ANGULAR_VELOCITY = 0.85 * Math.PI; 
    public static final double WHEELBASE_WIDTH = .762;
    public static final double WHEELBASE_LENGTH = .762;
    public static final double WHEEL_DIAMETER_METERS = .1016;


    private static final int FRONT_LEFT_DRIVE_PORT = 1;
    private static final int FRONT_RIGHT_DRIVE_PORT = 4;
    private static final int REAR_LEFT_DRIVE_PORT = 7;
    private static final int REAR_RIGHT_DRIVE_PORT = 10;

    private static final int FRONT_LEFT_TURN_PORT = 2;
    private static final int FRONT_RIGHT_TURN_PORT = 5;
    private static final int REAR_LEFT_TURN_PORT = 8;
    private static final int REAR_RIGHT_TURN_PORT = 11;

    private static final int FRONT_LEFT_ENCODER_PORT = 3;
    private static final int FRONT_RIGHT_ENCODER_PORT = 6;
    private static final int REAR_LEFT_ENCODER_PORT = 9;
    private static final int REAR_RIGHT_ENCODER_PORT = 12;

    private static final double FRONT_LEFT_ENCODER_OFFSET = 174.0234375 - 90;//174.0234375;//11.162109375;//2.724609375;
    private static final double FRONT_RIGHT_ENCODER_OFFSET =  -73.037109375 - 90;//176.1328125;//-110.830078125 ;// -111.263671875;
    private static final double REAR_LEFT_ENCODER_OFFSET = 1.58203125 - 90;//-4.74609375;//-6.328125;//6-2.263671875;
    private static final double REAR_RIGHT_ENCODER_OFFSET = -106.5234375 - 90;//173.759765625   ;//-79.716796875;//-80.439453125;

    public static SwerveModule FRONT_LEFT_MODULE = new SwerveModule(FRONT_LEFT_DRIVE_PORT,FRONT_LEFT_TURN_PORT,FRONT_LEFT_ENCODER_PORT,FRONT_LEFT_ENCODER_OFFSET, true);
    public static SwerveModule FRONT_RIGHT_MODULE = new SwerveModule(FRONT_RIGHT_DRIVE_PORT,FRONT_RIGHT_TURN_PORT,FRONT_RIGHT_ENCODER_PORT,FRONT_RIGHT_ENCODER_OFFSET, true);
    public static SwerveModule REAR_LEFT_MODULE = new SwerveModule(REAR_LEFT_DRIVE_PORT,REAR_LEFT_TURN_PORT,REAR_LEFT_ENCODER_PORT,REAR_LEFT_ENCODER_OFFSET, true);
    public static SwerveModule REAR_RIGHT_MODULE = new SwerveModule(REAR_RIGHT_DRIVE_PORT,REAR_RIGHT_TURN_PORT,REAR_RIGHT_ENCODER_PORT,REAR_RIGHT_ENCODER_OFFSET, false);

    public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);


    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION,REAR_LEFT_MODULE_POSITION,FRONT_RIGHT_MODULE_POSITION,REAR_RIGHT_MODULE_POSITION);

    public static final double TURN_kP = 3/2;
    public static final double TURN_kD = .015;

    public static final double AUTO_CONTROLLER_kP = 2;

    public static final SwerveModuleState[] LOCKED_SwerveModule_STATES = {new SwerveModuleState(0, Rotation2d.fromDegrees(45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(45))};

  }


    public static class ElevatorConstants{
      public static final double ELEVATOR_GEARING = 6;
      public static final int DEVICE_ID_ELEVATOR = 0;

    }

    public static class WristConstants{
      public static final double WRIST_GEARING = 12/60;
      public static final int DEVICE_ID_WRIST = 0;

      public static final double SOFT_LIMIT_FORWARD = 0;
      public static final double SOFT_LIMIT_REVERSE = -81094;

      public static final double WRIST_CONTROLLER_TOLERANCE_RAD = Units.degreesToRadians(2);

      public static final int ABS_ENCODER_PORT = 0;

      public static final double WRIST_CONTROLLER_KP = .75;
      public static final double WRIST_CONTROLLER_KI = 0;
      public static final double WRIST_CONTROLLER_KD = 0;
      public static final double WRIST_CONTROLLER_KF = 0;
    }
}
