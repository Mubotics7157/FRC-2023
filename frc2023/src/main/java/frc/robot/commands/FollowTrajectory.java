package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowTrajectory extends CommandBase {
    
    public synchronized double getAutoTime(){
        return Timer.getFPGATimestamp()-autoStartTime;
    }

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }


    private Drive drive;
    private Tracker tracker;

    private Trajectory path;
    private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    private ProfiledPIDController rotController = new ProfiledPIDController(-4, 0, 0,rotProfile);

    private PIDController xController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);
    private PIDController yController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);
    private Trajectory currTrajectory;
    private HolonomicDriveController autoController;    

    private double autoStartTime;

    public FollowTrajectory(Trajectory path, Drive instance, Tracker odom){
        this.path = path;
        drive = instance;
        tracker = odom;
        
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        autoController = new HolonomicDriveController(xController, yController, rotController); 
        autoController.setTolerance(new Pose2d(.5,.5,Rotation2d.fromDegrees(10)));

        addRequirements(drive,tracker);
    }
    @Override
    public void initialize() {
        tracker.setOdometry(currTrajectory.getInitialPose());
        rotController.reset(Tracker.getInstance().getOdometry().getRotation().getRadians());
        autoStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Trajectory.State goal = path.sample(getAutoTime());
        SmartDashboard.putNumber("desired rotation", goal.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("heading error",Math.abs(goal.poseMeters.getRotation().rotateBy(Tracker.getInstance().getOdometry().getRotation()).getDegrees()));
        SmartDashboard.putNumber("time elapsed", getAutoTime());

        ChassisSpeeds desiredSpeeds = autoController.calculate(Tracker.getInstance().getOdometry(), goal, goal.poseMeters.getRotation());

        driveFromChassis(desiredSpeeds);

        SmartDashboard.putNumber("total time", currTrajectory.getTotalTimeSeconds());

        if(autoController.atReference()&&getAutoTime()>= currTrajectory.getTotalTimeSeconds()){
            driveFromChassis(new ChassisSpeeds());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
