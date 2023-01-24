package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase{
    private static VisionManager instance = new VisionManager();

    PhotonCamera limeLight;
    PhotonPipelineResult limeResult;

    PhotonCamera shutter;
    PhotonPipelineResult shutterResult;
    
    AprilTagFieldLayout aprilTagFieldLayout;

    Transform3d robotToLime;
    Transform3d robotToShutter;

    PhotonPoseEstimator limePoseEstimator;
    PhotonPoseEstimator shutterPoseEstimator;
    

    public VisionManager(){
        limeLight = new PhotonCamera("limeLight");
        limeResult = limeLight.getLatestResult();
        
        shutter = new PhotonCamera("GSC");
        shutterResult = shutter.getLatestResult();

        try{
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        catch(Exception e){
            System.out.print("yes");
        }

        robotToShutter = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        robotToLime = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0));
        //^^^^^ needs to be configured!! this is the distance of your camera to the center of your robobot >:P 
        limePoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limeLight, robotToLime);
        shutterPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,shutter, robotToShutter);

    }

    @Override
    public void periodic() {
        updateResults();
    }

    public static VisionManager getInstance(){
        return instance;
    }

    public boolean limeHasTargets(){
        boolean hasTargets = limeResult.hasTargets();
        return hasTargets;
    }

    public boolean shutterHasTargets(){
        boolean hasTargets = shutterResult.hasTargets();
        return hasTargets;
    }

    public Rotation2d getLimeYaw(){
        if(limeHasTargets()){
            return Rotation2d.fromDegrees(limeResult.getBestTarget().getYaw());
        }
        else
            return Rotation2d.fromDegrees(0);
    }

    public Rotation2d getShutterYaw(){
        if(shutterHasTargets()){
            return Rotation2d.fromDegrees(shutterResult.getBestTarget().getYaw());
        }
        else
            return Rotation2d.fromDegrees(0);
    }

    public Rotation2d getLimePitch(){
        if(limeHasTargets()){
            return Rotation2d.fromDegrees(limeResult.getBestTarget().getPitch());
        }
        else
            return Rotation2d.fromDegrees(0);
    } 

    public Rotation2d getShutterPitch(){
        if(shutterHasTargets()){
            return Rotation2d.fromDegrees(shutterResult.getBestTarget().getPitch());
            }
        else
            return Rotation2d.fromDegrees(0);
    }

    public double getLimeLatency(){
        return limeResult.getTimestampSeconds();
    }

    public double getShutterLatency(){
        return shutterResult.getTimestampSeconds();
    }

    public Optional<EstimatedRobotPose> getEstimatedLimePose(Pose2d prevEstimatedRobotPose) {
        limePoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return limePoseEstimator.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedShutterPose(Pose2d prevEstimatedRobotPose) {
        shutterPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return shutterPoseEstimator.update();
    }

    public void updateResults(){
        limeResult = limeLight.getLatestResult();
        shutterResult = shutter.getLatestResult();
    }

    public Pose2d getLimePose(){
        EstimatedRobotPose estPose = getEstimatedLimePose(Tracker.getInstance().getOdometry()).get();
        double x = estPose.estimatedPose.getX();
        double y = estPose.estimatedPose.getY();
        Rotation2d theta = Rotation2d.fromRadians(estPose.estimatedPose.getRotation().getAngle());

        Pose2d pose = new Pose2d(x, y, theta);
        return pose;
    }

    public Pose2d getShutterPose(){
        EstimatedRobotPose estPose = getEstimatedShutterPose(Tracker.getInstance().getOdometry()).get();
        double x = estPose.estimatedPose.getX();
        double y = estPose.estimatedPose.getY();
        Rotation2d theta = Rotation2d.fromRadians(estPose.estimatedPose.getRotation().getAngle());

        Pose2d pose = new Pose2d(x, y, theta);
        return pose;
    }



}   
