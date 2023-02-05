package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase{
    
    NetworkTable tableLime;
    private double[] testArray = {0, 0, 0, 0, 0, 0};
    private static VisionManager instance = new VisionManager();
    private double offset;

    public VisionManager(){
        tableLime = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static VisionManager getInstance(){
        return instance;
    }


    @Override
    public void periodic() {
        logData();
    }
    

    public Rotation2d getTargetYaw(){
        double targets = tableLime.getEntry("tv").getDouble(0);
        double yaw = tableLime.getEntry("tx").getDouble(0);
        if(targets != 0){
            return Rotation2d.fromDegrees(yaw);
        }
        else
            return Rotation2d.fromDegrees(0);
    }

    public double getTargetPitch(){
        double targets = tableLime.getEntry("tv").getDouble(0);
        double pitch = tableLime.getEntry("ty").getDouble(0);
        if(targets != 0){
            return pitch;
        }
        else
            return 0;
    }

    public double getTargets(){
        double targets = tableLime.getEntry("tv").getDouble(0);
        return targets;
    }

    public boolean hasTargets(){
        double targets = tableLime.getEntry("tv").getDouble(0);

        if(targets == 0){
            return false;
        }
        else
            return true;
    }

    public void toggleLimeLight(){
        if(tableLime.getEntry("ledMode").getDouble(0) == 1){
            tableLime.getEntry("ledMode").setNumber(1);
        }
        else if(tableLime.getEntry("ledMode").getDouble(0) == 0){
            tableLime.getEntry("ledMode").setNumber(1);
        }
    }

    public void togglePipeLine(){
        //switch to object detection to reflective tape
        if(tableLime.getEntry("pipeline").getDouble(0) == 1){
            tableLime.getEntry("pipeline").setDouble(0);
        }
        else if(tableLime.getEntry("pipeline").getDouble(0) == 0){
            tableLime.getEntry("pipeline").setDouble(1);
        }
    }

    public void changePipeline(int index){
        tableLime.getEntry("pipeline").setDouble(index);
    }

    public double getPipelineIndex(){
        return tableLime.getEntry("pipeline").getDouble(0);
    }
    
    public void logData(){
        SmartDashboard.putNumber("Target yaw", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Target pitch", getTargetPitch());
        SmartDashboard.putNumber("Targets", getTargets());
        SmartDashboard.putNumber("offset", offset);
    
    }

    public Pose2d getBotPose(){
        /* 
        double[] poseEntry = tableLime.getEntry("botpose").getDoubleArray(testArray);
        if(hasTargets()&& poseEntry.length>0){
        
        Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[4]));
        return pose;
        }
        else
            return new Pose2d();
            */
        if(hasTargets()){
            try{
                double[] poseEntry = tableLime.getEntry("botpose").getDoubleArray(testArray);
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
        double latency = tableLime.getEntry("tl").getDouble(0);
        return latency;
    }

    public double saveObjectOffset(){
        offset = getTargetYaw().getDegrees();
        return offset;
    }

    public double getOffset(){
        return offset;
    }


}   