package frc.robot.subsystems;

import java.util.TreeMap;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants.BlueConstants;
import frc.robot.Constants.FieldConstants.RedConstants;
import frc.robot.util.Limelight;

public class VisionManager extends SubsystemBase{
    
    private static VisionManager instance = new VisionManager();
    private Limelight targetLL;
    private Limelight intakeLL;
    
    private MedianFilter coneFilter;
    private double coneOffset;
    private TreeMap<Double,Double> offsetMap = new TreeMap<>();

    private TreeMap<Double, Double> CubeNodeMap = new TreeMap<>();
    private TreeMap<Double, Double> ConeNodeMap = new TreeMap<>();

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


    public void addFieldRelativePose(){
        if(targetLL.hasTargets()) //TODO: add a check to make sure we are in tags mode
            Tracker.getInstance().addVisionMeasurement(targetLL.getBotPose(),targetLL.getLatency());
        
    }

    public Rotation2d getOffset(){
        return Rotation2d.fromDegrees(coneOffset);
    }

    public Rotation2d getAdjustedOffset(){

        return Rotation2d.fromDegrees(targetLL.getTargetYaw().getDegrees());
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
        addFieldRelativePose();
    }

    public void logData(){
        SmartDashboard.putNumber("Intake Target Yaw", getConeOffset());
        SmartDashboard.putNumber("Intake offset", coneOffset);
        SmartDashboard.putNumber("distance to target", getDistanceToTarget());
        SmartDashboard.putNumber("Vision Pose X", targetLL.getBotPose().getX());
        SmartDashboard.putNumber("Vision Pose Y", targetLL.getBotPose().getY());
        SmartDashboard.putNumber("Vision Pose R", targetLL.getBotPose().getRotation().getDegrees());
        try{
        SmartDashboard.putNumber("Cone Pose X", getIntakeConePose().getX());
        SmartDashboard.putNumber("Cone Pose Y", getIntakeConePose().getY());
        SmartDashboard.putNumber("Cone Pose R", getIntakeConePose().getRotation().getDegrees());

        SmartDashboard.putNumber("Interpolated Cone Horizontal Distance", getOffset().getDegrees());
        }
        catch(Exception e ){

        }
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
        
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_1.getY(), BlueConstants.NODE_CONE_BLUE_1.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_2.getY(), BlueConstants.NODE_CONE_BLUE_2.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_3.getY(), BlueConstants.NODE_CONE_BLUE_3.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_4.getY(), BlueConstants.NODE_CONE_BLUE_4.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_5.getY(), BlueConstants.NODE_CONE_BLUE_5.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_6.getY(), BlueConstants.NODE_CONE_BLUE_6.getY());

        ConeNodeMap.put(RedConstants.NODE_CONE_RED_1.getY(), RedConstants.NODE_CONE_RED_1.getY());
        ConeNodeMap.put(RedConstants.NODE_CONE_RED_2.getY(), RedConstants.NODE_CONE_RED_2.getY());
        ConeNodeMap.put(RedConstants.NODE_CONE_RED_3.getY(), RedConstants.NODE_CONE_RED_3.getY());
        ConeNodeMap.put(RedConstants.NODE_CONE_RED_4.getY(), RedConstants.NODE_CONE_RED_4.getY());
        ConeNodeMap.put(RedConstants.NODE_CONE_RED_5.getY(), RedConstants.NODE_CONE_RED_5.getY());
        ConeNodeMap.put(RedConstants.NODE_CONE_RED_6.getY(), RedConstants.NODE_CONE_RED_6.getY());
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

    public Pose2d getIntakeConePose(){
        try{
            return Tracker.getInstance().getPose().transformBy(new Transform2d(new Translation2d(VisionConstants.CAM_DIST_TO_INTAKE,getDistanceToTarget()), Rotation2d.fromDegrees(0)));
        }
        catch(Exception e ){
            System.out.print("==========COULD NOT GRAB CONE POSE=======");
            return Tracker.getInstance().getPose();
        }
    }


}   