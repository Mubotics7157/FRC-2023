package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.VisionManager.VisionState;

public class AlignRotation extends CommandBase {

    private PIDController rotController = new PIDController(2,0,0);
    private Drive drive;
    private double deltaSpeed;
    private Rotation2d angle;
    private DoubleSupplier fwd, str;

    public AlignRotation(Drive instance, DoubleSupplier fwd, DoubleSupplier str){
        drive = instance;
        this.fwd = fwd;
        this.str = str;
        angle = new Rotation2d();

        rotController.setTolerance(Units.degreesToRadians(1));
        addRequirements(drive);

        SmartDashboard.putNumber("align rotation P", 2);
    }
    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

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

    @Override
    public void initialize() {
        angle = Drive.getInstance().getSoftAngle();
        
        rotController.setP(SmartDashboard.getNumber("align rotation P", 2));

        if(DriverStation.getAlliance() == Alliance.Blue){
            angle = angle.unaryMinus();
        }

        rotController.reset();
        rotController.setSetpoint(angle.getRadians());
    }
    

    @Override
    public void execute() {
            if(!(rotController.atSetpoint()) )
                deltaSpeed = rotController.calculate(drive.getDriveHeading().getRadians());
            else
                rotController.calculate(angle.getRadians());
            
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(
                modifyInputs(-fwd.getAsDouble(), false),
                modifyInputs(-str.getAsDouble(), false),
                deltaSpeed*(DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 4), 
                Tracker.getInstance().getPose().getRotation())
                );
        
    }
    /* 
    @Override
    public boolean isFinished() {
        return strafeController.atSetpoint();
    }
    */

    @Override
    public void end(boolean interrupted) {
        //vision.setTargetLLState(VisionState.TAG);
    }

    
}

