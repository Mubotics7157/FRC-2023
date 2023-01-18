package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class DriveRobot extends CommandBase{
    
    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val) < .15){
                val = 0;
            }
            return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        else{
            if(Math.abs(val) < .1){
                val = 0;
            }
            return val * DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    private DoubleSupplier fwd, str, rot;
    private Drive drive;

    public DriveRobot(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation, Drive instance){
        fwd = forward;
        str = strafe;
        rot = rotation;

        drive = instance;

        addRequirements(drive);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double vx = modifyInputs(fwd.getAsDouble(), false);
        double vy =  modifyInputs(str.getAsDouble(),false);
        double omega = modifyInputs(rot.getAsDouble(), true);

        driveFromChassis(new ChassisSpeeds(vx, vy, omega));
    }
}