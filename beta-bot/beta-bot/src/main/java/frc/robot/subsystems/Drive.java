package frc.robot.subsystems;



import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import frc.robot.util.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

    private double driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    private double driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    private double tanDeadband = 0.15;
    private double angDeadband = 0.15;

    private Alliance lastKnownAlliance;

    private static Drive instance = new Drive();
    private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,Constants.DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
    private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,Constants.DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
    private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,Constants.DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
    private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,Constants.DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
    
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON,DriveConstants.CANIVORE_NAME);
    private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    private ProfiledPIDController rotController = new ProfiledPIDController(.5, 0, 0,rotProfile);



    public Drive(){
        rotController.setTolerance(5);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();

        SmartDashboard.putNumber("align P", 0.25);
        SmartDashboard.putNumber("strafe P", 0.25);
        SmartDashboard.putNumber("offset strafe", 0);

        PathPlannerServer.startServer(5811);
    }

    public static Drive getInstance(){
        if(instance == null){
            return new Drive();
        }
        else
            return instance;
    }

    @Override
    public void periodic() {
        //logData();
        SmartDashboard.putNumber("Drive Speed", getTan());
    }
    
    public void logData(){
         
        SmartDashboard.putNumber("left front angle", frontLeft.getHeading().getDegrees());
        SmartDashboard.putNumber("left rear angle", rearLeft.getHeading().getDegrees());
        SmartDashboard.putNumber("right front angle", frontRight.getHeading().getDegrees());
        SmartDashboard.putNumber("right rear angle", rearRight.getHeading().getDegrees());

        
        SmartDashboard.putNumber("left front velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("left rear velocity", rearLeft.getDriveVelocity());
        SmartDashboard.putNumber("right front velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("right rear velocity", rearRight.getDriveVelocity());

        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());

    }

    public void setModuleStates(SwerveModuleState[] states){
        
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

        SmartDashboard.putNumber("front left drive setpoint", states[0].speedMetersPerSecond);
    }
    
    public Rotation2d getDriveHeading(){
        return gyro.getRotation2d();
    }

    public double getDrivePitch(){
        return gyro.getPitch();
    }

    public void resetHeading(){
        Tracker.getInstance().resetHeading();    
    }
    
    public void changeMax(){
        tanDeadband = 0.15;
        angDeadband = 0.15;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    }

    public void changeSlow(){
        tanDeadband = 0.20;
        angDeadband = 0.25;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 3;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 3;
    }

    public void changeVerySlow(){
        tanDeadband = 0.20;
        angDeadband = 0.25;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 5;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 5;
    }

    public double getTan(){
        return driveSpeed;
    }

    public double getAng(){
        return driveAngle;
    }

    public double getTanDeadband(){
        return tanDeadband;
    }

    public double getAngDeadband(){
        return angDeadband;
    }
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = {frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};

        return modulePositions;
    }

    public ProfiledPIDController getRotationController(){
        return rotController;
    }

    public void lockModules(){
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public void changeMotorGains(){
        frontLeft.changeDriveKP();
        frontRight.changeDriveKP();
        rearLeft.changeDriveKP();
        rearRight.changeDriveKP();

        frontLeft.changeTurnKP();
        frontRight.changeTurnKP();
        rearLeft.changeTurnKP();
        rearRight.changeTurnKP();
        
    }

    public PPSwerveControllerCommand followPath(PathPlannerTrajectory traj,boolean startingPath){
            traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

            if(startingPath)
                Tracker.getInstance().setPose(traj.getInitialHolonomicPose());
  
            return new PPSwerveControllerCommand(
                 traj,
                 Tracker.getInstance()::getPose, 
                 DriveConstants.DRIVE_KINEMATICS, 
                 new PIDController(2.5
                 , 0, 0),
                 new PIDController(2.5, 0, 0), 
                 new PIDController(1.75, 0, 0), 
                 this::setModuleStates,  
                 false,
                 this
            );
    }
        public PPSwerveControllerCommand followPath(Pose2d starting,double distMeters){
            Pose2d offsetPose = starting.plus(new Transform2d(new Translation2d(distMeters, 0),Rotation2d.fromDegrees(0)));
            PathPlannerTrajectory traj = PathPlanner.generatePath(
                new com.pathplanner.lib.PathConstraints(Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY, DriveConstants.MAX_DRIVE_TANGENTIAL_ACCEL),
                new PathPoint(starting.getTranslation(),starting.getRotation(),starting.getRotation()), 
                new PathPoint(offsetPose.getTranslation(), offsetPose.getRotation(),starting.getRotation()) 
            );
            traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

            Tracker.getInstance().setPose(traj.getInitialHolonomicPose());
            return new PPSwerveControllerCommand(
                 traj,
                 Tracker.getInstance()::getPose, 
                 DriveConstants.DRIVE_KINEMATICS, 
                 new PIDController(2.5
                 , 0, 0),
                 new PIDController(2.5, 0, 0),
                 new PIDController(1.75, 0, 0), 
                 this::setModuleStates,  
                 false,
                 this
            );
    }

    public Alliance getLastAlliance(){
        return lastKnownAlliance;
    }

    public void setLastAlliance(Alliance alliance){
        lastKnownAlliance = alliance;
    }

    private void stop(){
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }
}