package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(),
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
        .02,
        .02,
        .01
    ),
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
        .1,
        .1,
        .05
    ));


    private final Field2d m_field = new Field2d();

    private static Tracker instance = new Tracker();

    public Tracker(){
        SmartDashboard.putData("Field", m_field);
    }

    public  void updatePose(){
        
        estimator.updateWithTime(Timer.getFPGATimestamp(),Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
    }

    public void addVisionMeasurement(Pose2d visionPose,double latency){
        if(Math.max(Math.abs(visionPose.getX()-getPose().getX()),Math.abs(visionPose.getY()-getPose().getY()))>1)
            estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp()-latency);
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
        updatePose();

        SmartDashboard.putNumber("estim x", getPose().getX());
        SmartDashboard.putNumber("estim y", getPose().getY());
        SmartDashboard.putNumber("estim r", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("dist", getDistanceToTarget());
    }

    public void setOdometry(Pose2d pose){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public Pose2d getOdometry(){
        return estimator.getEstimatedPosition();
    }

    public void resetHeading(){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(estimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
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

    public void resetViaVision(){
        Pose2d visionPose = VisionManager.getInstance().getBotPose();
        if(visionPose!=null)
            estimator.resetPosition(visionPose.getRotation(),Drive.getInstance().getModulePositions(),visionPose);
    }
}
