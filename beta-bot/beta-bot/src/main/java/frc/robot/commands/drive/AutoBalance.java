package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;

public class AutoBalance extends CommandBase{
    Drive drive; 
    VisionManager vision;
    ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.5));
    double lastKnownDistance;
    DoubleSupplier str, rot;

    public AutoBalance(DoubleSupplier str, DoubleSupplier rot, Drive drive, VisionManager vision){
        this.drive = drive; 
        this.vision = vision;

        this.rot = rot;
        this.str = str;
        lastKnownDistance = 0;
        addRequirements(drive);

        controller.setGoal(2.85);
        controller.setTolerance(.1);
    }

    @Override
    public void initialize() {
        drive.changeSlow();
    }

    @Override
    public void execute() {
        double vy = modifyInputs(str.getAsDouble(), false);
        double omega = modifyInputs(rot.getAsDouble(), true);

        try{
            lastKnownDistance = vision.getDistanceToTag();
        }
        catch(Exception e){
            
        }

        double fwdSpeed = controller.calculate(lastKnownDistance);

        driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(fwdSpeed, -vy, omega, Tracker.getInstance().getPose().getRotation()));

    }

    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val)<drive.getAngDeadband()){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
            return val*drive.getAng();
        }
        else{
            if(Math.abs(val)<drive.getTanDeadband()){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
            return val*drive.getTan();
        }
    }
    
    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }
}
