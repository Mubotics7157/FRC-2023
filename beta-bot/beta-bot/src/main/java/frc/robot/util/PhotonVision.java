package frc.robot.util;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
//ohhhhh cause itsss youuuuuuuuuuuuuuuu that I lieeeeee withhhh
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Tracker;

public class PhotonVision {
    
    private String name;
    private Transform3d location;

    private PhotonCamera camera;

    private PhotonPoseEstimator photonEstimator;

    private AprilTagFieldLayout aprilTagFieldLayout;

    public PhotonVision(String name, Transform3d location){
        this.name = name;
        this.location = location;

        camera = new PhotonCamera(name);

        try{
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        catch(Exception e){
            System.out.print("how did we get here?");
        }

        photonEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            camera,
            location);
    }

    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }

    public Rotation2d getTargetYaw(){
        if(hasTargets())
            return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getYaw());
        else
            throw new NullPointerException();
    }

    public Rotation2d getTargetPitch(){
        if(hasTargets())
            return Rotation2d.fromDegrees(camera.getLatestResult().getBestTarget().getPitch());
        else
            throw new NullPointerException();
    }

    public double getLatency(){
        return camera.getLatestResult().getLatencyMillis();
    }

    public double getTimestamp(){
        if(hasTargets()){
            return getEstimatedShutterPose().get().timestampSeconds;
        }
        else{
            throw new NullPointerException();
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedShutterPose() {
        //only to be used inside class :P
        photonEstimator.setReferencePose(Tracker.getInstance().getOdometry());
        return photonEstimator.update();
    }

    public Pose2d getBotPose(){
        if(hasTargets()){
            Pose2d pose = getEstimatedShutterPose().get().estimatedPose.toPose2d();
            return pose;
        }
        else{
            throw new NullPointerException();
        }
    }

    
    
}
