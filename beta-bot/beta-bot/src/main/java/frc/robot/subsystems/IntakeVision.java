package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeVision extends SubsystemBase{
    
    private NetworkTable tableLime;

    private static IntakeVision instance = new IntakeVision();

    private double offset;
    //private MedianFilter filter = new MedianFilter(20);

    public IntakeVision(){
        tableLime = NetworkTableInstance.getDefault().getTable("limelight-intake");
    }

    public static IntakeVision getInstance(){
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

    public Rotation2d getTargetPitch(){
        double targets = tableLime.getEntry("tv").getDouble(0);
        double pitch = tableLime.getEntry("ty").getDouble(0);
        if(targets != 0){
            return Rotation2d.fromDegrees(pitch);
        }
        else
            return Rotation2d.fromDegrees(0);
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
        SmartDashboard.putNumber("Intake Target yaw", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Intake Target pitch", getTargetPitch().getDegrees());
        SmartDashboard.putNumber("Intake Targets", getTargets());
        SmartDashboard.putNumber("Intake offset", offset);
    
    }

    public double getLatency(){
        double latency = tableLime.getEntry("tl").getDouble(0);
        return latency;
    }

    public void setObjectOffset(){
        offset = getTargetYaw().getDegrees() * IntakeConstants.OFFSET_COEFFICIENT;
        //return offset;
    }

    public double getOffset(){
        return offset;
    }


}   