package frc.robot.subsystems;

import java.time.OffsetDateTime;

import javax.sound.midi.Track;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.PPSwerveControllerCommand;

public class Tracker extends SubsystemBase{

    private Pose2d node = FieldConstants.RedConstants.NODE_CONE_RED_1;
    private double coneOffset = 0;
    private PathPlannerTrajectory traj;
    private final Field2d m_field = new Field2d();
    private static Tracker instance = new Tracker();
    private SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            .1,
            .1,
            .1
        ),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill( //vision boi
            15,
            15,
            15
        )
    );

    public static Tracker getInstance(){
        return instance;
    }

    public Tracker(){
        resetViaVision();
        initTunableFields();
        SmartDashboard.putNumber("custom offset", 0);
    }


    @Override
    public void periodic() {
        //editNodePose();
        updatePose();
        m_field.setRobotPose(estimator.getEstimatedPosition());
        node = new Pose2d(new Translation2d(FieldConstants.BlueConstants.NODE_CONE_BLUE_1.getX(), getPose().getY()), Rotation2d.fromDegrees(0));

        //SmartDashboard.putNumber("estim x", getPose().getX());
        //SmartDas1hboard.putNumber("estim y", getPose().getY());
        m_field.getObject("gyro").setPose(new Pose2d(getPose().getTranslation(),Drive.getInstance().getDriveHeading()));
        SmartDashboard.putNumber("estim r", getPose().getRotation().getDegrees());
    }

    private void updatePose(){
        
        estimator.updateWithTime(Timer.getFPGATimestamp(),Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
    }

    public void addVisionMeasurement(Pose2d visionPose,double latency){
        if(Math.max(Math.abs(visionPose.getX()-getPose().getX()),Math.abs(visionPose.getY()-getPose().getY()))<1)
            estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp()-latency);
    }

    public void regeneratePath(){
        //man i love polar bears
        Translation2d offsetPose = new Translation2d(node.getX(), node.getY() + coneOffset);
        if(DriverStation.getAlliance() == Alliance.Red)
            traj = PathPlanner.generatePath(
                new com.pathplanner.lib.PathConstraints(2, 3),
                new PathPoint(Tracker.getInstance().getPose().getTranslation(), Tracker.getInstance().getPose().getRotation(),Tracker.getInstance().getPose().getRotation()), // position, heading
                new PathPoint(offsetPose,Tracker.getInstance().getPose().getRotation(),Rotation2d.fromDegrees(180)) // position, heading
            );
        else
            traj = PathPlanner.generatePath(
                new com.pathplanner.lib.PathConstraints(2, 3),
                new PathPoint(Tracker.getInstance().getPose().getTranslation(), Tracker.getInstance().getPose().getRotation()), // position, heading
                new PathPoint(offsetPose,Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(180)) // position, heading
            );
    }

    public void setOffset(){
        coneOffset = SmartDashboard.getNumber("custom offset", 0);
    }

    public void editNodePose(double nodeY){
        node = new Pose2d(new Translation2d(SmartDashboard.getNumber("Node X", FieldConstants.RedConstants.NODE_CONE_RED_2.getX()), nodeY), Rotation2d.fromDegrees(0));//new Pose2d(new Translation2d(SmartDashboard.getNumber("Node X", 14.25), VisionManager.getInstance().getNodeY()), Rotation2d.fromDegrees(0));
        m_field.getObject("pose").setPose(node);
    }

    public void setPose(Pose2d pose){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public void setOdometry(Pose2d pose){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public Pose2d getPose(){
        return estimator.getEstimatedPosition();
    }

    public Command getPathFollowingCommand(){
       
       return new PPSwerveControllerCommand(
           getTraj(), 
           this::getPose, 
           DriveConstants.DRIVE_KINEMATICS,
           new PIDController(5, 0, 0), 
           new PIDController(5, 0, 0),
           new PIDController(SmartDashboard.getNumber("r val", 2.5), 0, 0), 
           Drive.getInstance()::setModuleStates,
           false, 
           Drive.getInstance()
           );
    }

    public PathPlannerTrajectory getTraj(){
        return traj;
    }

    public void resetViaVision(){
        Pose2d visionPose = VisionManager.getInstance().getBotPose();
        if(visionPose!=null)
            estimator.resetPosition(Drive.getInstance().getDriveHeading(),Drive.getInstance().getModulePositions(),visionPose);
    }

    public void resetHeading(){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(estimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
    }

    private void initTunableFields(){
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("xy val", 2.5);
        SmartDashboard.putNumber("r val", 2.5);
        SmartDashboard.putNumber("Node X", FieldConstants.RedConstants.NODE_CONE_RED_2.getX());
        SmartDashboard.putNumber("Node Y", FieldConstants.RedConstants.NODE_CONE_RED_2.getY());
        m_field.getObject("pose").setPose(node);

    }

}
