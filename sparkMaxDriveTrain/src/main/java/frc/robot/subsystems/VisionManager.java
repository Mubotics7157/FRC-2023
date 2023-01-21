package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase{
    
    NetworkTable tableLime;
    private double[] testArray = {0, 0, 0, 0, 0, 0};
    private static VisionManager instance = new VisionManager();

    public VisionManager(){
        tableLime = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static VisionManager getInstance(){
        return instance;
    }

    public double getTargetYaw(){
        double targets = tableLime.getDoubleTopic("tv").subscribe(0).get();
        double yaw = tableLime.getDoubleTopic("tx").subscribe(0).get();
        if(targets != 0){
            return yaw;
        }
        else
            return 0;
    }

    public double getTargetPitch(){
        double targets = tableLime.getDoubleTopic("tv").subscribe(0).get();
        double pitch = tableLime.getDoubleTopic("ty").subscribe(0).get();
        if(targets != 0){
            return pitch;
        }
        else
            return 0;
    }

    public double getTargets(){
        double targets = tableLime.getDoubleTopic("tv").subscribe(0).get();
        return targets;
    }

    public boolean hasTargets(){
        double targets = tableLime.getDoubleTopic("tv").subscribe(0).get();

        if(targets == 0){
            return false;
        }
        else
            return true;
    }

    public void toggleLimeLight(){
        if(tableLime.getDoubleTopic("ledMode").subscribe(0).get() == 1){
            tableLime.getDoubleTopic("ledMode").publish().set(0);
        }
        else if(tableLime.getDoubleTopic("ledMode").subscribe(0).get() == 0){
            tableLime.getDoubleTopic("ledMode").publish().set(1);
        }
    }

    public void togglePipeLine(){
        //switch to object detection to reflective tape
        if(tableLime.getDoubleTopic("pipeline").subscribe(0).get() == 1){
            tableLime.getDoubleTopic("pipeline").publish().set(0);;
        }
        else if(tableLime.getDoubleTopic("pipeline").subscribe(0).get() == 0){
            tableLime.getDoubleTopic("pipeline").publish().set(1);
        }
    }
    
    public void logData(){
        SmartDashboard.putNumber("Target yaw", getTargetYaw());
        SmartDashboard.putNumber("Target pitch", getTargetPitch());
        SmartDashboard.putNumber("Targets", getTargets());
    
    }

    public Pose2d getBotPose(){
        /* 
        double[] poseEntry = tableLime.getDoubleTopic("botpose").getDoubleArray(testArray);

        if(hasTargets()&& poseEntry.length>0){
        
        Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[4]));
        return pose;
        }
        else
            return new Pose2d();
            */
        if(hasTargets()){
            try{
                double[] poseEntry = tableLime.getDoubleArrayTopic("botpose").subscribe(testArray).get();
                Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[5]));
                return pose;
            }
            catch(Exception e){
                return new Pose2d();
            }
        }
        else
            return new Pose2d();
    }

    public double getLatency(){
        double latency = tableLime.getDoubleTopic("tl").subscribe(0).get();
        return latency;
    }
}   
