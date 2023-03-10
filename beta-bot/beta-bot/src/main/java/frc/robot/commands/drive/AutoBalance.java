package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;

public class AutoBalance extends CommandBase{
    Drive drive; 
    //ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(0.75, 0.75));
    //double lastKnownDistance;

    public AutoBalance(Drive drive){
        this.drive = drive; 

        //lastKnownDistance = 0;
        addRequirements(drive);

        //controller.setGoal(2.75);
        //controller.setTolerance(.05);

        //SmartDashboard.putNumber("custom balance P", 1);
        //SmartDashboard.putNumber("custom constraints", 0.75);
        //SmartDashboard.putNumber("custom goal", 2.85);
        SmartDashboard.putNumber("custom threshold", 2);
    }

    @Override
    public void initialize() {
        /* 
        try{
            lastKnownDistance = VisionManager.getInstance().getDistanceToTag();
            SmartDashboard.putNumber("last known dist", lastKnownDistance);
        }
        catch(Exception e){
            
        }
        
        controller.reset(lastKnownDistance);
        controller.setP(SmartDashboard.getNumber("custom balance P", 1));
        controller.setGoal(SmartDashboard.getNumber("custom goal", 2.75));
        controller.setConstraints(new TrapezoidProfile.Constraints(SmartDashboard.getNumber("custom constraints", 0.25), SmartDashboard.getNumber("custom constraints", 0.25)));
        */
    }

    @Override
    public void execute() {
        if(Math.abs(drive.getDrivePitch()) > 2)
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY * 0.1, 0,0, Tracker.getInstance().getPose().getRotation()));

        /* 
        SmartDashboard.putBoolean("auto balance at goal?", controller.atGoal());
        SmartDashboard.putNumber("auto error", controller.getPositionError());
        SmartDashboard.putNumber("auto setpoint", controller.getSetpoint().position);

        try{
            lastKnownDistance = VisionManager.getInstance().getDistanceToTag();
            SmartDashboard.putNumber("last known dist", lastKnownDistance);
        }
        catch(Exception e){
            
        }

        double fwdSpeed = controller.calculate(lastKnownDistance);
        SmartDashboard.putNumber("controller output", fwdSpeed);

        if(!controller.atSetpoint())
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(fwdSpeed*DriveConstants.MAX_TELE_ANGULAR_VELOCITY * 0.5, 0,0, Tracker.getInstance().getPose().getRotation()));

        else if(controller.atSetpoint())
            drive.lockModules();
            */

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(drive.getDrivePitch()) < SmartDashboard.getNumber("custom threshold", 2);//controller.atGoal();
    }


    @Override
    public void end(boolean interrupted) {
        driveFromChassis(new ChassisSpeeds());

        drive.lockModules();
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
