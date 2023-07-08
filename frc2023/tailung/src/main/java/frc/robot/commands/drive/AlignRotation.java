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

public class AlignRotation extends CommandBase {

    private PIDController rotController = new PIDController(1.25,0,0);
    private Drive drive;
    private double deltaSpeed;
    private Rotation2d angle;
    private Rotation2d angleToAlign;
    private DoubleSupplier fwd, str;

    public AlignRotation(DoubleSupplier fwd, DoubleSupplier str, Rotation2d angle){
        drive = Drive.getInstance();
        this.fwd = fwd;
        this.str = str;
        this.angle = angle;

        addRequirements(drive);
        rotController.setTolerance(Units.degreesToRadians(1));
        rotController.isContinuousInputEnabled();
        rotController.enableContinuousInput(0, Units.degreesToRadians(360));

        SmartDashboard.putNumber("align rotation P", 1.25);

        rotController.setP(SmartDashboard.getNumber("align rotation P", 1.25));
        
        SmartDashboard.putNumber("angle to rotate", angle.getDegrees());
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
        
        rotController.setP(SmartDashboard.getNumber("align rotation P", 1.25));

        if(DriverStation.getAlliance() == Alliance.Blue){
            angleToAlign = angle.unaryMinus();
        }
        else
            angleToAlign = angle;
         

        rotController.reset();
        rotController.setSetpoint(angleToAlign.getRadians());
        
        SmartDashboard.putNumber("angle to rotate", angleToAlign.getDegrees());
    }
    

    @Override
    public void execute() {
            if(!(rotController.atSetpoint()))
                deltaSpeed = rotController.calculate(Tracker.getInstance().getPose().getRotation().getRadians());
            else
                rotController.calculate(angleToAlign.getRadians());
            
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(
                modifyInputs(-fwd.getAsDouble(), false),
                modifyInputs(-str.getAsDouble(), false),
                deltaSpeed*(DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 2), 
                Tracker.getInstance().getPose().getRotation())
                );
        
    }
    
    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }
    

    
}

