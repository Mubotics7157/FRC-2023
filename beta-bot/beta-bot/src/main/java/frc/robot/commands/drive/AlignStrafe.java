package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.stream.Stream;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;

public class AlignStrafe extends CommandBase{
    private Drive drive;
    private Tracker tracker;
    private VisionManager vision;
    private ProfiledPIDController rotController;
    private ProfiledPIDController strController;
    private boolean atGoal;
    private double deltaSpeed;

    private double strSpeed;

    private DoubleSupplier fwd, str, rot;

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    public AlignStrafe(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive dInstance, Tracker tInstance, VisionManager vInstance){
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        
        drive = dInstance;
        tracker = tInstance;
        vision = vInstance;
        addRequirements(drive);
        addRequirements(tracker);
        addRequirements(vision);
    }

    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val)<.15){
                val = 0;
            }
            return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        else{
            if(Math.abs(val)<.1){
                val = 0;
            }
            return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }

    @Override
    public void initialize() {
        atGoal= false;
        rotController = new ProfiledPIDController(.3, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , 2*Math.PI));
        strController = new ProfiledPIDController(.3, 0, 0, new TrapezoidProfile.Constraints(3 * Math.PI, 4 * Math.PI));

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(Units.degreesToRadians(3));

        strController.enableContinuousInput(-Math.PI, Math.PI);
        strController.setTolerance(Units.degreesToRadians(3));

    }

    @Override
    public void execute() {

        double vx =  modifyInputs(fwd.getAsDouble(),false);
        double vy =  modifyInputs(str.getAsDouble(),false);
        double omega = modifyInputs(rot.getAsDouble(), true);
        
        Rotation2d onTarget = Rotation2d.fromDegrees(180);
        double error = onTarget.rotateBy(tracker.getOdometry().getRotation()).getRadians();

        Rotation2d strTarget = Rotation2d.fromDegrees(0);
        double strError = strTarget.rotateBy(vision.getTargetYaw()).getRadians();

        if(Math.abs(error)>Units.degreesToRadians(3))
            deltaSpeed = rotController.calculate(error);
        else{
            deltaSpeed =0;
            atGoal = true;
        }

        if(Math.abs(strError) > Units.degreesToRadians(3))
            strSpeed = -strController.calculate(strError);
        else
            strSpeed = 0;

        if(vision.hasTargets())
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, strSpeed*DriveConstants.MAX_TANGENTIAL_VELOCITY, deltaSpeed*DriveConstants.MAX_TELE_ANGULAR_VELOCITY, Tracker.getInstance().getOdometry().getRotation()));
        else
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, deltaSpeed * DriveConstants.MAX_TELE_ANGULAR_VELOCITY, Tracker.getInstance().getOdometry().getRotation()));
        
        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("strafe speed", strSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
        SmartDashboard.putNumber("strafe error", Units.degreesToRadians(strError));
        SmartDashboard.putBoolean("On target", rotController.atGoal());
    }

    /* 
    @Override
    public boolean isFinished() {
        return atGoal;
    }
    */
    @Override
    public void end(boolean interrupted) {
        driveFromChassis(new ChassisSpeeds());
    }
}