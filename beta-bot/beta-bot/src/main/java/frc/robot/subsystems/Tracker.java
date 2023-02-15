package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;

public class Tracker extends SubsystemBase{

    SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d());

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(),Drive.getInstance().getModulePositions());

    private final Field2d m_field = new Field2d();

    private static Tracker instance = new Tracker();

    public Tracker(){
        SmartDashboard.putData("Field", m_field);
        
        estimator.setVisionMeasurementStdDevs(
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                0.02, //x 
                0.02, //y
                0.01  //theta
            ));
    }

    public  void updatePose(){
        try{
            estimator.addVisionMeasurement(VisionManager.getInstance().getFieldRelativePose(), Timer.getFPGATimestamp()-VisionManager.getInstance().getTargetLatency());
        }
        catch(Exception e){
            //System.out.println("======Could not add vision measurement======");
        }

        try{
            estimator.addVisionMeasurement(VisionManager.getInstance().getFieldRelativePoseShutter(), VisionManager.getInstance().getPoseTimestampShutter());
        }
        catch(Exception e){
            //System.out.println("======Could not add vision measurement======");
        }

        estimator.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
    }

    public  void adjustDeviation(double x, double y, double r){
        estimator.setVisionMeasurementStdDevs(
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                x,
                y,
                r 
        ));
    }

    public static Tracker getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(estimator.getEstimatedPosition());
        odometry.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
        updatePose();

        SmartDashboard.putNumber("odom x", getOdometry().getX());
        SmartDashboard.putNumber("odom y", getOdometry().getY());
        SmartDashboard.putNumber("odometry r", getOdometry().getRotation().getDegrees());

        SmartDashboard.putNumber("estim x", getPose().getX());
        SmartDashboard.putNumber("estim y", getPose().getY());
        SmartDashboard.putNumber("estim r", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("dist", getDistanceToTarget());
    }

    public void setOdometry(Pose2d pose){
        odometry.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }

    public void resetHeading(){
        odometry.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public void resetPoseHeading(){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(estimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    public void plotAuto(Trajectory trajectory){
        m_field.getObject("traj").setTrajectory(trajectory);
    }

    public double getDistanceToTarget(){
        // if(DriverStation.getAlliance()==Alliance.Blue)
            // return VisionConstants.BLUE_NODE_POSITION.minus(getOdometry()).getX();
        // else
            return VisionConstants.RED_NODE_POSITION.minus(getOdometry()).getX();
    }
}
