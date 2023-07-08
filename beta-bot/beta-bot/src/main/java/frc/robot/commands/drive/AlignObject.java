package frc.robot.commands.drive;

import javax.sound.midi.Track;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.VisionManager.VisionState;

public class AlignObject extends CommandBase {

    private PIDController strafeController = new PIDController(2
    ,0,0);
    private Drive drive;
    private VisionManager vision;
    private double deltaSpeed;

    private Pose2d newPose;

    public AlignObject(Drive instance,VisionManager vision){
        drive = instance;
        this.vision = vision;

        strafeController.setTolerance(Units.degreesToRadians(2));
        addRequirements(drive,vision);
    }

    public AlignObject(Drive instance, VisionManager vision, Pose2d newPose){
        drive = instance;
        this.vision = vision;
        this.newPose= newPose;

        strafeController.setTolerance(Units.degreesToRadians(2));
        addRequirements(drive, vision);
    }
    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    @Override
    public void initialize() {
        strafeController.reset();
        strafeController.setSetpoint(Units.degreesToRadians(5));
    }
    

    @Override
    public void execute() {
            if(!(strafeController.atSetpoint()) )
                deltaSpeed = strafeController.calculate(vision.getCubeYaw().getRadians());
            else
                strafeController.calculate(Units.degreesToRadians(5));
            
            driveFromChassis(new ChassisSpeeds(0, deltaSpeed*3, 0));
        
    }

    @Override
    public boolean isFinished() {
        if(!vision.getTargetLL().hasTargets()){
        }
        return strafeController.atSetpoint() || !vision.getTargetLL().hasTargets();
    }

    @Override
    public void end(boolean interrupted) {

        if(newPose != null){
            Tracker.getInstance().setPose(new Pose2d(newPose.getX(), newPose.getY(), Tracker.getInstance().getPose().getRotation()));
        }

        //vision.setTargetLLState(VisionState.TAG);
    }

    
}

