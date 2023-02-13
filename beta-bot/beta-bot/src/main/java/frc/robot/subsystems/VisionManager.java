package frc.robot.subsystems;

import java.util.TreeMap;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Limelight;

public class VisionManager extends SubsystemBase{
    
    private static VisionManager instance = new VisionManager();
    private Limelight targetLL;
    private Limelight intakeLL;
    
    private MedianFilter coneFilter;
    private double coneOffset;
    private TreeMap<Double,Double> offsetMap = new TreeMap<>();
    private double lastKnownDistance = 0;

    public enum VisionState{
        TAG,
        TAPE,
        OFF
    }

    public VisionManager(){
        targetLL = new Limelight(VisionConstants.TARGET_LL_NAME);
        intakeLL = new Limelight(VisionConstants.INTAKE_LL_NAME);

        coneFilter = new MedianFilter(VisionConstants.FILTER_SAMPLE_WINDOW);
        coneOffset = 0;

        initAlignmentMap();
    }

    public static VisionManager getInstance(){
        return instance;
    }

    public Rotation2d getNodeAngle(){
        return targetLL.getTargetYaw();
    }

    public double getConeOffset(){
        try{
            return coneFilter.calculate(intakeLL.getTargetYaw().getDegrees());
        }
        catch(NullPointerException e){
            return 0;
        }
    }

    public Pose2d getFieldRelativePose(){
        return targetLL.getBotPose();
    }

    public Rotation2d getOffset(){
        return Rotation2d.fromDegrees(coneOffset);
    }

    public Rotation2d getAdjustedOffset(){
        double roundedOffset = Math.round(coneOffset);

        return Rotation2d.fromDegrees(offsetMap.get(roundedOffset));
    }

    public double getTargetLatency(){
        return targetLL.getLatency();
    }

    public boolean foundNode(){
        return targetLL.hasTargets();
    }


    @Override
    public void periodic() {
        logData();
        coneOffset = getConeOffset();
    }

    public void logData(){
        SmartDashboard.putNumber("Intake Target Yaw", getConeOffset());
        SmartDashboard.putNumber("Intake offset", coneOffset);
        //SmartDashboard.putNumber("Experimental Cone Offset", getAdjustedOffset().getDegrees());
    }

    public void setTargetLLState(VisionState state){
        switch(state){
            case TAG:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
                break;
            case TAPE:
                targetLL.setPipelineIndex(VisionConstants.TAPE_PIPELINE_INDEX);
                break;
            default:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
                break;
        }
    }

    public void togglePipeline(){
        targetLL.setPipelineIndex(targetLL.getPipelineIndex()==VisionConstants.TAPE_PIPELINE_INDEX? VisionConstants.TAG_PIPELINE_INDEX: VisionConstants.TAPE_PIPELINE_INDEX);
    }

    private void initAlignmentMap(){
        offsetMap.put(0.0, -5.2);
        offsetMap.put(-1.0, -5.2);
        offsetMap.put(-2.0, -5.2);
        offsetMap.put(-3.0, -5.2);
        offsetMap.put(-4.0, -5.2);
        offsetMap.put(-5.0, -5.2);
        offsetMap.put(-6.0, -5.2);
        offsetMap.put(-7.0, -5.2);
        offsetMap.put(-8.0, -5.2);
        offsetMap.put(-9.0, -5.2);
        offsetMap.put(-10.0, -5.2);
        offsetMap.put(-11.0, -5.2);
        offsetMap.put(-12.0, -5.2);
        offsetMap.put(-13.0, -5.2);
        offsetMap.put(-14.0, -5.2);
        offsetMap.put(-15.0, -5.2);
        offsetMap.put(-16.0, -5.2);
        offsetMap.put(-17.0, -5.2);

        offsetMap.put(1.0, 3.98);
        offsetMap.put(2.0, 3.98);
        offsetMap.put(3.0, 3.98);
        offsetMap.put(4.0, 3.98);
        offsetMap.put(5.0, 3.98);
        offsetMap.put(6.0, 3.98);
        offsetMap.put(7.0, 3.98);
        offsetMap.put(8.0, 3.98);
        offsetMap.put(9.0, 3.98);
        offsetMap.put(10.0,3.98);
        offsetMap.put(11.0,3.98);
        offsetMap.put(12.0,3.98);
        offsetMap.put(13.0,3.98);
        offsetMap.put(14.0,3.98);
        offsetMap.put(15.0,3.98);
        offsetMap.put(16.0,3.98);
        offsetMap.put(17.0,3.98);
    }

    public double getDistanceToTarget(){
        try{
            double targetPitch = targetLL.getTargetPitch().getRadians();
            double distance = (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.CAM_HEIGHT_METERS) / Math.tan(VisionConstants.CAM_MOUNTING_PITCH_RADIANS + targetPitch);
                if(distance<5){
                    lastKnownDistance = distance;
                    return distance;
                }
                else 
                    return lastKnownDistance;
        }
        catch(Exception e){
            return lastKnownDistance;
        }
 
    }

}   