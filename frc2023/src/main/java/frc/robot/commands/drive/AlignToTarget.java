package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

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

public class AlignToTarget extends CommandBase{
    private Drive drive;
    private VisionManager vision;
    private ProfiledPIDController controller;
    private boolean atGoal;
    private double deltaSpeed;

    private DoubleSupplier fwd, str, rot;

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    public AlignToTarget(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive dInstance, VisionManager vInstance){
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        drive = dInstance;
        vision = vInstance;
        addRequirements(drive);
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
        controller = new ProfiledPIDController(.3, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , 2*Math.PI));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Units.degreesToRadians(3));

    }

    @Override
    public void execute() {

        double vx =  modifyInputs(fwd.getAsDouble(),false);
        double vy =  modifyInputs(str.getAsDouble(),false);
        double omega = modifyInputs(rot.getAsDouble(), true);
        
        Rotation2d onTarget = Rotation2d.fromDegrees(0);
        double error = onTarget.rotateBy(vision.getTargetYaw()).getRadians();


        if(Math.abs(error)>Units.degreesToRadians(3))
            deltaSpeed = controller.calculate(error);
        else{
            deltaSpeed =0;
            atGoal = true;
        }

        if(vision.hasTargets()){
        driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, deltaSpeed*DriveConstants.MAX_TELE_ANGULAR_VELOCITY, Tracker.getInstance().getOdometry().getRotation()));
        }
        else
        driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Tracker.getInstance().getOdometry().getRotation()));

        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
        SmartDashboard.putBoolean("On target", controller.atGoal());
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
