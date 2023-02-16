package frc.robot.subsystems;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drive.PathConstraints;

public class Tracker extends SubsystemBase{

    Pose2d node = FieldConstants.RedConstants.NODE_CONE_RED_2;
    private PathPlannerTrajectory traj;
    SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(),
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
        .02,
        .02,
        .01
    ),
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
        .2,
        .2,
        .2
    ));

    private SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          this::getPose, // Pose2d supplier
          this::setOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
          AutoConstants.X_Y_CONTROLLER,
          AutoConstants.ROT_CONTROLLER,
          Drive.getInstance()::setModuleStates, // Module states consumer used to output to the drive subsystem
          new HashMap<>(),
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          Drive.getInstance() // The drive subsystem. Used to properly set the requirements of path following commands
          ); 

    private final Field2d m_field = new Field2d();

    private static Tracker instance = new Tracker();

    public Tracker(){

        SmartDashboard.putData("Field", m_field);

        SmartDashboard.putNumber("x deviation", 0.02);
        SmartDashboard.putNumber("y deviation", 0.02);
        SmartDashboard.putNumber("r deviation", 0.02);
    SmartDashboard.putNumber("Node X", FieldConstants.RedConstants.NODE_CONE_RED_2.getX());
    SmartDashboard.putNumber("Node Y", FieldConstants.RedConstants.NODE_CONE_RED_2.getY());
    }

    public void regeneratePath(){
        traj = PathPlanner.generatePath(
            new com.pathplanner.lib.PathConstraints(.5,.5),
            new PathPoint(Tracker.getInstance().getPose().getTranslation(), Tracker.getInstance().getPose().getRotation()), // position, heading
            new PathPoint(FieldConstants.RedConstants.NODE_CONE_RED_2.getTranslation(),Rotation2d.fromDegrees(0)) // position, heading
        );
    }

     public Command getCommand(){
        
        return new PPSwerveControllerCommand(
            getTraj(), 
            this::getPose, // Pose supplier
            DriveConstants.DRIVE_KINEMATICS,
            new PIDController(1.25, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(1.25, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(2.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            Drive.getInstance()::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            Drive.getInstance()
            );
    }

    public PathPlannerTrajectory getTraj(){
        return traj;
    }

    private void editNodePose(){
        node = new Pose2d(new Translation2d(SmartDashboard.getNumber("Node X", FieldConstants.RedConstants.NODE_CONE_RED_2.getX()),SmartDashboard.getNumber("Node Y", FieldConstants.RedConstants.NODE_CONE_RED_2.getY())), Rotation2d.fromDegrees(0));
    }

    
    public  void updatePose(){
        
        estimator.updateWithTime(Timer.getFPGATimestamp(),Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
    }

    public void addVisionMeasurement(Pose2d visionPose,double latency){
        if(Math.max(Math.abs(visionPose.getX()-getPose().getX()),Math.abs(visionPose.getY()-getPose().getY()))<1)
            estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp()-latency);
    }

    public  void adjustDeviation(){
        // estimator.setVisionMeasurementStdDevs(
            // new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
                // SmartDashboard.getNumber("x deviation", 0.02),
                // SmartDashboard.getNumber("y deviation", 0.02),
                // SmartDashboard.getNumber("r deviation", 0.02)
        // ));
    }

    public static Tracker getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        editNodePose();
        m_field.setRobotPose(estimator.getEstimatedPosition());
        updatePose();

        SmartDashboard.putNumber("estim x", getPose().getX());
        SmartDashboard.putNumber("estim y", getPose().getY());
        SmartDashboard.putNumber("estim r", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("dist", getDistanceToTarget());
        m_field.getObject("pose").setPose(node);
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
