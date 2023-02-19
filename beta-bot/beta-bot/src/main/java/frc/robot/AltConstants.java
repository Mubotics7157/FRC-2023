package frc.robot;


import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class AltConstants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class SwerveModuleConstants{
    public static final double driveKS = 0.71003;
    public static final double driveKV = 2.2783;
    public static final double driveKA = 0.25953;
    public static final double driveKP = 0.003;

    public static final SimpleMotorFeedforward DRIVE_FEEDFORWARD = new SimpleMotorFeedforward(driveKS,driveKV,driveKA);
    public static final double TURNING_KP = .2;

    public static final double CLOSED_LOOP_RAMP_RATE  = .2;
    public static final double OPEN_LOOP_RAMP_RATE  = .25;

    public static final double TURN_GEAR_RATIO  = 21.34;
    public static final double DRIVE_GEAR_RATIO  = 6.75;

    public static final String SWERVE_CANIVORE_ID = "swerve";

    public static final int TIMEOUT_MS = 50;
}

  public static class DriveConstants{
    public static final double MAX_TANGENTIAL_VELOCITY = 3.5; 
    public static final double MAX_TELE_TANGENTIAL_VELOCITY = 3.5; 
    public static final double MAX_TELE_ANGULAR_VELOCITY = 2 * Math.PI; 
    public static final double WHEELBASE_WIDTH = .6604;
    public static final double WHEELBASE_LENGTH = .6604;
    public static final double WHEEL_DIAMETER_METERS = .1016;


    public static final int REAR_RIGHT_DRIVE_PORT = 10;
    public static final int FRONT_RIGHT_DRIVE_PORT = 7;
    public static final int FRONT_LEFT_DRIVE_PORT = 4;
    public static final int REAR_LEFT_DRIVE_PORT = 1;

    public static final int REAR_RIGHT_TURN_PORT = 11;
    public static final int FRONT_RIGHT_TURN_PORT = 8;
    public static final int FRONT_LEFT_TURN_PORT = 5;
    public static final int REAR_LEFT_TURN_PORT = 2;

    public static final int REAR_RIGHT_ENCODER_PORT = 12;
    public static final int FRONT_RIGHT_ENCODER_PORT = 9;
    public static final int FRONT_LEFT_ENCODER_PORT = 6;
    public static final int REAR_LEFT_ENCODER_PORT = 3;

    public static final double REAR_RIGHT_ENCODER_OFFSET = 10.107010016401127;//-97.3;//178.857;//174.0234375;//11.162109375;//2.724609375;
    public static final double FRONT_RIGHT_ENCODER_OFFSET =  127.0007175492034;//-167;//-4.219;//176.1328125;//-110.830078125 ;// -111.263671875;
    public static final double FRONT_LEFT_ENCODER_OFFSET = -49.65367268041238;//-168.1;//-169.277;//-4.74609375;//-6.328125;//6-2.263671875;
    public static final double REAR_LEFT_ENCODER_OFFSET = -154.42226306232428;//-107;

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

      public static final double ELEVATOR_HEIGHT_TOLERANCE = 1.5;

      public static final double ZEROING_SPEED = -.05;

      public static final double ELEVATOR_ZERO_HEIGHT = 0;

      public static int DEVICE_ID_ELEVATOR_SWITCH = 0;
      //TODO: find out what id to use

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

      public static final int DEVICE_ID_MAG_SENSOR = 1;
      //TODO: find out what the id is


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

      public static final int LIMELIGHT_ON = 0;
      public static final int LIMELIGHT_OFF = 1;

      public static final int TAPE_PIPELINE_INDEX = 1;
      public static final int TAG_PIPELINE_INDEX = 0;
      public static final int CONE_PIPELINE_INDEX = 0;

      public static final String INTAKE_LL_NAME = "limelight-intake";
      public static final String TARGET_LL_NAME = "limelight";//"limelight-polecam";

      public static final int FILTER_SAMPLE_WINDOW = 20;

      public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(46);
      public static final double CAM_MOUNTING_PITCH_RADIANS = Units.degreesToRadians(-15);
      public static final double CAM_HEIGHT_METERS = Units.inchesToMeters(46.5);

      public static final double CAM_DIST_TO_INTAKE = 0;
 
    }

    public static class IntakeConstants{
      public static final int DEVICE_ID_INTAKE_SLAVE = 21;
      public static final int DEVICE_ID_INTAKE_MASTER = 20;

      public static final int DEVICE_ID_SOLENOID_FORWARD = 0;
      public static final int DEVICE_ID_SOLENOID_REVERSE = 1;

      public static final int DEVICE_ID_PCM = 28;

      public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

      public static final int ULTRASONIC_PING_PORT = 0;
      public static final int ULTRASONIC_RESPONSE_PORT = 1;

      public static final int FILTER_SAMPLE_WINDOW = 40;

      public static final double CUBE_INTAKE_SPEED = .25;
      public static final double CONE_INTAKE_SPEED = .75;

      public static final double CONE_OUTTAKE_SPEED=-0.9;
      public static final double CUBE_OUTTAKE_SPEED=-0.9;
      public static final double CUBE_INTAKE_SETPOINT= .25;
      public static final double CONE_INTAKE_SETPOINT= .75;
      public static final double CONE_OUTTAKE_SETPOINT=-30123;
      public static final double CUBE_OUTTAKE_SETPOINT=-0.9;
      public static final double IDLE_SETPOINT = 0;

      public static final double IDLE_SPEED = .65;

      public static final double TOP_ROLLER_KP = 0.00007;
      public static final double TOP_ROLLER_KF = .000186;
      public static final double BOT_ROLLER_KF = .000332;
      public static final double BOT_ROLLER_KP = .00007;

      public static final boolean INTAKE_MASTER = true;


    }

    public static final class LidarConstants {
      public static final double CALIBRATION_OFFSET = 0;
      public static final int DIO_PORT = 0;
      public static final double ANGLE_OFFSET = 2;
  }

  public static final class AutoConstants{
    //TODO: load path jsons and event maps for relevant auto routines here!!!!

    public static final PIDConstants X_Y_CONTROLLER = new PIDConstants(1.25, 0, 0);
    public static final PIDConstants ROT_CONTROLLER = new PIDConstants(2.5, 0, 0);
  }

  public static final class SuperStructureConstants{
    public static final double ELEVATOR_CONE_HIGH = -26;
    public static final Rotation2d WRIST_CONE_HIGH = Rotation2d.fromDegrees(-107);

    public static final double ELEVATOR_CONE_MID = -15;
    public static final Rotation2d WRIST_CONE_MID = Rotation2d.fromDegrees(-135);

    public static final double ELEVATOR_INTAKE_CONE_FALLEN = 0;
    public static final Rotation2d WRIST_INTAKE_CONE_FALLEN = Rotation2d.fromDegrees(-123);

    public static final double ELEVATOR_INTAKE_CONE_UPRIGHT = -4;
    public static final Rotation2d WRIST_INTAKE_CONE_UPRIGHT = Rotation2d.fromDegrees(-135);

    public static final double ELEVATOR_INTAKE_CUBE = 0;
    public static final Rotation2d WRIST_INTAKE_CUBE = Rotation2d.fromDegrees(-123);

    public static final double ELEVATOR_CUBE_HIGH = 0;
    public static final Rotation2d WRIST_CUBE_HIGH = Rotation2d.fromDegrees(-40);
  }

  public static final class FieldConstants{

      public static final class RedConstants{
        public static final Pose2d NODE_CONE_RED_1 = new Pose2d(14.25, 4.95, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_RED_2 = new Pose2d(14.25, 3.85, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_RED_3 = new Pose2d(14.25, 3.30, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_RED_4 = new Pose2d(14.25, 2.18, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_RED_5 = new Pose2d(14.25, 1.61, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_RED_6 = new Pose2d(14.25, 0.45, Rotation2d.fromDegrees(0));

        public static final Pose2d NODE_CUBE_RED_1 = new Pose2d(14.25, 4.42, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CUBE_RED_2 = new Pose2d(14.25, 2.74, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CUBE_RED_3 = new Pose2d(14.25, 1, Rotation2d.fromDegrees(0));
      }
      
      public static final class BlueConstants{
        public static final Pose2d NODE_CONE_BLUE_1 = new Pose2d(1.75, 4.95, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_BLUE_2 = new Pose2d(1.75, 3.85, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_BLUE_3 = new Pose2d(1.75, 3.30, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_BLUE_4 = new Pose2d(1.75, 2.18, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_BLUE_5 = new Pose2d(1.75, 1.61, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CONE_BLUE_6 = new Pose2d(1.75, 0.45, Rotation2d.fromDegrees(0));

        public static final Pose2d NODE_CUBE_BLUE_1 = new Pose2d(1.75, 4.42, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CUBE_BLUE_2 = new Pose2d(1.75, 2.74, Rotation2d.fromDegrees(0));
        public static final Pose2d NODE_CUBE_BLUE_3 = new Pose2d(1.75, 1, Rotation2d.fromDegrees(0));
      }

  }
}
