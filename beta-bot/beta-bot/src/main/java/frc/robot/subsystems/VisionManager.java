package frc.robot.subsystems;

import java.util.TreeMap;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants.BlueConstants;
import frc.robot.util.Limelight;

public class VisionManager extends SubsystemBase{
    
    private static VisionManager instance = new VisionManager();
    private Limelight targetLL;
    private Limelight intakeLL;
    private MedianFilter coneFilter;
    private double coneOffset;
    private boolean useVision = true;
    private double lastTimeStamp = 0;
    private TreeMap<Double, Double> ConeNodeMap = new TreeMap<>();
    private double lastKnownDistance = 0;

    public enum VisionState{
        TAG,
        CUBE,
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

    @Override
    public void periodic() {
        //logData();
        addFieldRelativePose();
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

    public Pose2d getBotPose(){
        if(targetLL.hasTargets() && targetLL.getPipelineIndex()==VisionConstants.TAG_PIPELINE_INDEX) 
            return targetLL.getBotPose();
        else    
            return null;
        
    }
    
    public double getDistanceToTag(){
        if(targetLL.hasTargets()){
            if(LimelightHelpers.getFiducialID(VisionConstants.TARGET_LL_NAME) == 7 || LimelightHelpers.getFiducialID(VisionConstants.TARGET_LL_NAME) == 2){
                return LimelightHelpers.getTargetPose3d_CameraSpace(VisionConstants.TARGET_LL_NAME).getTranslation().getNorm();
            }
            else
                throw new NullPointerException();
       
        }
        else
            throw new NullPointerException();
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

    public double getNodeY(){
            return ConeNodeMap.get(Tracker.getInstance().getPose().getY());
    }

    public Limelight getTargetLL(){
        return targetLL;
    }

    public Pose2d getIntakeConePose(){
        // try{
            // return Tracker.getInstance().getPose().transformBy(new Transform2d(new Translation2d(VisionConstants.CAM_DIST_TO_INTAKE,getDistanceToTarget()), Rotation2d.fromDegrees(0)));
        // }
        // catch(Exception e ){
            // System.out.print("==========COULD NOT GRAB CONE POSE=======");
            // return Tracker.getInstance().getPose();
        // }
        return Tracker.getInstance().getPose();
    }

    public void addFieldRelativePose(){
        if(Math.abs(Drive.getInstance().getDrivePitch()) < 1){
            try{
            if(targetLL.hasTargets()) //&& (targetLL.getBootTimeStamp()-lastKnownDistance) > 1000) 
                Tracker.getInstance().addVisionMeasurement(targetLL.getBotPose(),targetLL.getLatency());
           // if(intakeLL.hasTargets()) //&& (targetLL.getBootTimeStamp()-lastKnownDistance) > 1000) 
                //Tracker.getInstance().addVisionMeasurement(intakeLL.getBotPose(),intakeLL.getLatency());
        //lastTimeStamp = targetLL.getBootTimeStamp();
            }
            catch(Exception e){

            }
        }
    }

    public void useVision(){
        useVision = true;
    }

    public void noUseVision(){
        useVision = false;
    }


    public void togglePipeline(){
        targetLL.setPipelineIndex(targetLL.getPipelineIndex()==VisionConstants.CUBE_PIPELINE_INDEX? VisionConstants.TAG_PIPELINE_INDEX: VisionConstants.CUBE_PIPELINE_INDEX);
    }

    public void setTargetLLState(VisionState state){
        switch(state){
            case TAG:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
                break;
            case CUBE:
                targetLL.setPipelineIndex(VisionConstants.CUBE_PIPELINE_INDEX);
                break;
            default:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
                break;
        }
    }

    public VisionState getTargetLLPipelineState(){
        switch((int)targetLL.getPipelineIndex()){
            case 0:
                return VisionState.TAG;
            case 1:
                return VisionState.CUBE;
            default:
                return VisionState.TAG;
        }
    }


    private void initAlignmentMap(){
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_1.getY(), BlueConstants.NODE_CONE_BLUE_1.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_2.getY(), BlueConstants.NODE_CONE_BLUE_2.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_3.getY(), BlueConstants.NODE_CONE_BLUE_3.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_4.getY(), BlueConstants.NODE_CONE_BLUE_4.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_5.getY(), BlueConstants.NODE_CONE_BLUE_5.getY());
        ConeNodeMap.put(BlueConstants.NODE_CONE_BLUE_6.getY(), BlueConstants.NODE_CONE_BLUE_6.getY());

        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_1.getY(), RedConstants.NODE_CONE_RED_1.getY());
        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_2.getY(), RedConstants.NODE_CONE_RED_2.getY());
        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_3.getY(), RedConstants.NODE_CONE_RED_3.getY());
        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_4.getY(), RedConstants.NODE_CONE_RED_4.getY());
        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_5.getY(), RedConstants.NODE_CONE_RED_5.getY());
        //ConeNodeMap.put(RedConstants.NODE_CONE_RED_6.getY(), RedConstants.NODE_CONE_RED_6.getY());
    }

    public void logData(){

        //SmartDashboard.putNumber("Vision Pose X", targetLL.getBotPose().getX());
        //SmartDashboard.putNumber("Vision Pose Y", targetLL.getBotPose().getY());
        //SmartDashboard.putNumber("Vision Pose R", targetLL.getBotPose().getRotation().getDegrees());
        
        //SmartDashboard.putNumber("Cone Pose X", getIntakeConePose().getX());
        //SmartDashboard.putNumber("Cone Pose Y", getIntakeConePose().getY());
        //SmartDashboard.putNumber("Cone Pose R", getIntakeConePose().getRotation().getDegrees());

        //SmartDashboard.putNumber("Interpolated Cone Horizontal Distance", getOffset().getDegrees());

        try{
            SmartDashboard.putNumber("Cube Yaw", getCubeYaw().getDegrees());
        }
        catch(Exception e){

        }


    }

    public Rotation2d getCubeYaw(){
        if(targetLL.getPipelineIndex()==0){
            try{
                return targetLL.getTargetYaw();
            }
            catch(Exception e){
                return Rotation2d.fromDegrees(7);
            }
        }
        else
            return Rotation2d.fromDegrees(7);
    }
}   