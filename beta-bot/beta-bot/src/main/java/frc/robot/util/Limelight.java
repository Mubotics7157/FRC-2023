package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

public class Limelight {
    private String name;
    private NetworkTable tableLime;

    public Limelight(String name){
        this.name = name; 
        tableLime = NetworkTableInstance.getDefault().getTable(name);
    }

    public boolean hasTargets(){
        return tableLime.getEntry("tv").getDouble(0) ==1;
    }

    public void setPipelineIndex(int on){
        tableLime.getEntry("pipeline").setDouble(on);
    }

    public void setLEDs(boolean on){
        if(on)
            tableLime.getEntry("ledMode").setNumber(VisionConstants.LIMELIGHT_ON);
        else
            tableLime.getEntry("ledMode").setNumber(VisionConstants.LIMELIGHT_OFF);

    }

    public Rotation2d getTargetYaw(){
        if(hasTargets())
            return Rotation2d.fromDegrees(tableLime.getEntry("tx").getDouble(0));
        else{
            throw new NullPointerException();
        }
    }

    public Rotation2d getTargetPitch(){
        if(hasTargets())
            return Rotation2d.fromDegrees(tableLime.getEntry("ty").getDouble(0));
        else{
            throw new NullPointerException();
        }
    }

    public Pose2d getBotPose(){
        if(hasTargets()){
            try{
                double[] poseEntry = tableLime.getEntry("botpose").getDoubleArray(new double[6]);
                Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[5]));
                return pose;
            }
            catch(Exception e){
                System.out.println("=========Error Adding New Bot Pose========");
                throw new NullPointerException();
            }
        }

        else{
            throw new NullPointerException();
        }
    }

    public double getLatency(){
        double latency = tableLime.getEntry("tl").getDouble(0) + tableLime.getEntry("cl").getDouble(0);
        return latency;
    }

    public double getTargets(){
        return tableLime.getEntry("tv").getDouble(0);
    }

    public double getPipelineIndex(){
        return tableLime.getEntry("pipeline").getDouble(0);
    }

    public String getPipelineName(){
        return name;
    }


}
