// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

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
    public static final double MAX_TELE_ANGULAR_VELOCITY = 2 * Math.PI; 
    public static final double WHEELBASE_WIDTH = .6604;
    public static final double WHEELBASE_LENGTH = .6604;
    public static final double WHEEL_DIAMETER_METERS = .1016;


    public static final int REAR_RIGHT_DRIVE_PORT = 1;
    public static final int FRONT_RIGHT_DRIVE_PORT = 4;
    public static final int FRONT_LEFT_DRIVE_PORT = 7;
    public static final int REAR_LEFT_DRIVE_PORT = 10;

    public static final int REAR_RIGHT_TURN_PORT = 2;
    public static final int FRONT_RIGHT_TURN_PORT = 5;
    public static final int FRONT_LEFT_TURN_PORT = 8;
    public static final int REAR_LEFT_TURN_PORT = 11;

    public static final int REAR_RIGHT_ENCODER_PORT = 3;
    public static final int FRONT_RIGHT_ENCODER_PORT = 6;
    public static final int FRONT_LEFT_ENCODER_PORT = 9;
    public static final int REAR_LEFT_ENCODER_PORT = 12;

    public static final double REAR_RIGHT_ENCODER_OFFSET = -97.3;//178.857;//174.0234375;//11.162109375;//2.724609375;
    public static final double FRONT_RIGHT_ENCODER_OFFSET =  -167;//-4.219;//176.1328125;//-110.830078125 ;// -111.263671875;
    public static final double FRONT_LEFT_ENCODER_OFFSET = -168.1;//-169.277;//-4.74609375;//-6.328125;//6-2.263671875;
    public static final double REAR_LEFT_ENCODER_OFFSET = -107;

    public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_LEFT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,WHEELBASE_LENGTH/2);
    public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);
    public static final Translation2d REAR_RIGHT_MODULE_POSITION = new Translation2d(-WHEELBASE_WIDTH/2,-WHEELBASE_LENGTH/2);

    public static final int DEVICE_ID_PIGEON = 30;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_MODULE_POSITION,FRONT_RIGHT_MODULE_POSITION,REAR_LEFT_MODULE_POSITION,REAR_RIGHT_MODULE_POSITION);

    public static final double TURN_kP = 3/2;
    public static final double TURN_kD = .015;

    public static final double AUTO_CONTROLLER_kP = 2;

    public static final SwerveModuleState[] LOCKED_SwerveModule_STATES = {new SwerveModuleState(0, Rotation2d.fromDegrees(45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),new SwerveModuleState(0, Rotation2d.fromDegrees(45))};

  }


    public static class ElevatorConstants{
      public static final double ELEVATOR_GEARING = 6;
      public static final int DEVICE_ID_ELEVATOR_MASTER = 10;
      public static final int DEVICE_ID_ELEVATOR_SLAVE = 11;

      public static final double WRIST_PEAK_OUTPUT_FORWARD = .75;
      public static final double WRIST_PEAK_OUTPUT_REVERSE = -.75;

      public static final double ELEVATOR_KP = .08;

      public static final double ELEVATOR_HEIGHT_TOLERANCE = 0;

      public static final double ZEROING_SPEED = -.05;

      public static final double ELEVATOR_ZERO_HEIGHT = 0;

    }

    public static class WristConstants{
      public static final double WRIST_GEARING = 60;
      public static final int DEVICE_ID_WRIST = 32;

      public static final double SOFT_LIMIT_FORWARD = 72456;
      public static final double SOFT_LIMIT_REVERSE = 0;

      public static final double WRIST_PEAK_OUTPUT_FORWARD = 1;
      public static final double WRIST_PEAK_OUTPUT_REVERSE = -1;

      public static final double WRIST_CONTROLLER_TOLERANCE_RAD = Units.degreesToRadians(2);

      public static final int ABS_ENCODER_PORT = 0;


      public static final double WRIST_CONTROLLER_KP = .075;
      public static final double WRIST_CONTROLLER_KI = 0;
      public static final double WRIST_CONTROLLER_KD = 0;
      public static final double WRIST_CONTROLLER_KF = 0;

      public static final double WRIST_KS = .13938;
      public static final double WRIST_KV = 1.08;
      public static final double WRIST_KG = 1.45;
      public static final double WRIST_KA = .0020997;


      public static final ArmFeedforward ARM_FF = new ArmFeedforward(WRIST_KS, WRIST_KG, WRIST_KV);
    }

    public static class VisionConstants{
      public static final double LIME_TO_INTAKE_METERS = 0.25;
      //^^^ number is arbitrary
      public static final Transform3d SHUTTER_TRANS = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
      public static final Transform3d LIME_TRANS = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
      //TODO: figure out the translations cuz its 0, 0, 0 rn soooooo :P
 
    }

    public static class IntakeConstants{
      public static final int DEVICE_ID_INTAKE_SLAVE = 21;
      public static final int DEVICE_ID_INTAKE_MASTER = 20;

      public static final int DEVICE_ID_SOLENOID_FORWARD = 6;
      public static final int DEVICE_ID_SOLENOID_REVERSE = 7;

      public static final int DEVICE_ID_REV_PH = 9;
    }

public static class FieldConstants{
  public static final AprilTag tag01= new AprilTag(
    1,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI))
  );

  public static final AprilTag tag02 = new AprilTag(
    2,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI))
  );

  public static final AprilTag tag03 = new AprilTag(
    3,
          new Pose3d(
              Units.inchesToMeters(610.77),
              Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
              Units.inchesToMeters(18.22),
              new Rotation3d(0.0, 0.0, Math.PI))
  );

  public static final AprilTag tag04 = new AprilTag(
    4,
    new Pose3d(
        Units.inchesToMeters(636.96),
        Units.inchesToMeters(265.74),
        Units.inchesToMeters(27.38),
        new Rotation3d(0.0, 0.0, Math.PI))
  );

  public static final AprilTag tag05 = new AprilTag(
    5,
          new Pose3d(
              Units.inchesToMeters(14.25),
              Units.inchesToMeters(265.74),
              Units.inchesToMeters(27.38),
              new Rotation3d())
  );

  public static final AprilTag tag06 = new AprilTag(
    6,
    new Pose3d(
        Units.inchesToMeters(40.45),
        Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
        Units.inchesToMeters(18.22),
        new Rotation3d())
  );

  public static final AprilTag tag07 = new AprilTag(
    7,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(108.19),
              Units.inchesToMeters(18.22),
              new Rotation3d())
  );

  public static final AprilTag tag08 = new AprilTag(
    8,
          new Pose3d(
              Units.inchesToMeters(40.45),
              Units.inchesToMeters(42.19),
              Units.inchesToMeters(18.22),
              new Rotation3d())
  );
}


}