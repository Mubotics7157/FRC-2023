package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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


    @Override
    public void periodic() {
        logData();
        coneOffset = getConeOffset();
    }

    public void logData(){
        SmartDashboard.putNumber("Intake Target Yaw", getConeOffset());
        SmartDashboard.putNumber("Intake Targets", intakeLL.getTargets());
        SmartDashboard.putNumber("Intake offset", coneOffset);
    }

    public void setTargetLLState(VisionState state){
        switch(state){
            case TAG:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
            case TAPE:
                targetLL.setPipelineIndex(VisionConstants.TAPE_PIPELINE_INDEX);
            default:
                targetLL.setPipelineIndex(VisionConstants.TAG_PIPELINE_INDEX);
        }
    }
    
}   