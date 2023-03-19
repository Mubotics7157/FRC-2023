package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class DriveTele extends CommandBase {

    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val)<drive.getAngDeadband()){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val*drive.getAng();
        }
        else{
            if(Math.abs(val)<drive.getTanDeadband()){
                val = 0;
            }
            //return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
            //val = Math.copySign(Math.pow(val, 2),val);
            return val*drive.getTan();
        }
    }
    
    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }
    private DoubleSupplier fwd,str,rot;
    private Drive drive;

    public DriveTele(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot,Drive instance){
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        drive = instance;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        //drive.changeMax();
    }

    @Override
    public void execute() {
        double vx =  modifyInputs(fwd.getAsDouble(),false);
        double vy =  modifyInputs(str.getAsDouble(),false);
        double omega = modifyInputs(rot.getAsDouble(), true);

        driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Tracker.getInstance().getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveFromChassis(new ChassisSpeeds());
    }


}
