package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;

public class Limelight {
    private String name;
    private NetworkTable tableLime;
    private double visionLatency= 0;

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
        double[] poseEntry;
        if(DriverStation.getAlliance()==Alliance.Red)
            poseEntry = tableLime.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        else
            poseEntry = tableLime.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        Pose2d pose = new Pose2d(poseEntry[0], poseEntry[1], Rotation2d.fromDegrees(poseEntry[5]));
        visionLatency = poseEntry[6];
        return pose;

    }

    public double getLatency(){
        return visionLatency *.001;
    }

    public double getTargets(){
        return tableLime.getEntry("tv").getDouble(0);
    }

    public LimelightResults getJsonDump(){
        return LimelightHelpers.getLatestResults(name);
    }

    public double getBootTimeStamp(){
        return getJsonDump().targetingResults.timestamp_LIMELIGHT_publish;
    }

    public double getPipelineIndex(){
        return tableLime.getEntry("pipeline").getDouble(0);
    }

    public String getPipelineName(){
        return name;
    }


}
