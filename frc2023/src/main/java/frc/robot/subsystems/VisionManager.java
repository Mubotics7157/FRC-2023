package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionManager extends SubsystemBase{
    
    NetworkTable tableLime;

    public VisionManager(){
        tableLime = NetworkTableInstance.getDefault().getTable("limelight");
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

    public double getPitch(){
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
        if(targets != 0){
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
    
    public void logData(){
        SmartDashboard.putNumber("Target yaw", getTargetYaw().getDegrees());
        SmartDashboard.putNumber("Target pitch", getPitch());
        SmartDashboard.putNumber("Targets", getTargets());
    
    }
    public double getPoleOffset(){
        //gets the offset of the pole to the game object on the intake (to account for error)
        return 0;
    }
}
