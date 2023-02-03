package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
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
import frc.robot.Constants.VisionConstants;

public class VisionManager extends SubsystemBase{
    private static VisionManager instance = new VisionManager();

    PhotonCamera limeLight;
    PhotonPipelineResult limeResult;

    //PhotonCamera shutter;
    //PhotonPipelineResult shutterResult;
    
    AprilTagFieldLayout aprilTagFieldLayout;

    //PhotonPoseEstimator limePoseEstimator;
    //PhotonPoseEstimator shutterPoseEstimator;
    

    public VisionManager(){
        limeLight = new PhotonCamera("limeLight");
        limeResult = limeLight.getLatestResult();
        
        // shutter = new PhotonCamera("GSC");
        // shutterResult = shutter.getLatestResult();

        try{
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        catch(Exception e){
            System.out.print("yes");
        }

        // limePoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, limeLight, VisionConstants.LIME_TRANS);
        // shutterPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE,shutter, VisionConstants.SHUTTER_TRANS);
        //TODO: ^^^^^ needs to be configured!! this is the distance of your camera to the center of your robobot >:P
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

    /*
    public boolean shutterHasTargets(){
        boolean hasTargets = shutterResult.hasTargets();
        return hasTargets;
    }
    */

    public Rotation2d getLimeYaw(){
        if(limeHasTargets()){
            return Rotation2d.fromDegrees(limeResult.getBestTarget().getYaw());
        }
        else
            return Rotation2d.fromDegrees(0);
    }

    /*public Rotation2d getShutterYaw(){
        if(shutterHasTargets()){
            return Rotation2d.fromDegrees(shutterResult.getBestTarget().getYaw());
        }
        else
            return Rotation2d.fromDegrees(0);
    }*/

    public Rotation2d getLimePitch(){
        if(limeHasTargets()){
            return Rotation2d.fromDegrees(limeResult.getBestTarget().getPitch());
        }
        else
            return Rotation2d.fromDegrees(0);
    } 

    /*public Rotation2d getShutterPitch(){
        if(shutterHasTargets()){
            return Rotation2d.fromDegrees(shutterResult.getBestTarget().getPitch());
            }
        else
            return Rotation2d.fromDegrees(0);
    }*/

    public double getLimeLatency(){
        return limeResult.getTimestampSeconds();
    }
    /* 
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
     */
    public void updateResults(){
        limeResult = limeLight.getLatestResult();
        //shutterResult = shutter.getLatestResult();
    }

    public void toggleLED(){
        if(limeLight.getLEDMode() == VisionLEDMode.kOff)
            limeLight.setLED(VisionLEDMode.kOn);
        else 
            limeLight.setLED(VisionLEDMode.kOff);
    }
    /* 
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
    */

    public void changePipeLine(int pipeline){
        //figure numbers out later
        limeLight.setPipelineIndex(pipeline);
    }
    public double getPoleOffset(){
        int oldPipe = limeLight.getPipelineIndex();

        changePipeLine(1);
        //whatever the pipeline index is for color
        double offset = Math.tan(VisionConstants.LIME_TO_INTAKE_METERS * getLimeYaw().getRadians());

        limeLight.setPipelineIndex(oldPipe);
        return offset;
    }

    public double configureOffset(){
        return Intake.getInstance().getScoringOffset();
    }





}   
