package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FollowTrajectory extends CommandBase {
    
    public synchronized double getAutoTime(){
        return Timer.getFPGATimestamp()-autoStartTime;
    }


    private Drive drive;
    private Trajectory path;
    TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    ProfiledPIDController rotController = new ProfiledPIDController(-4, 0, 0,rotProfile);

    PIDController xController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);
    PIDController yController = new PIDController(DriveConstants.AUTO_CONTROLLER_kP, 0, 0);
    Trajectory currTrajectory;
    private volatile Rotation2d desiredAutoHeading;
    HolonomicDriveController autoController;    

    private double autoStartTime;

    public FollowTrajectory(Trajectory path, Drive instance){
        this.path = path;
        drive = instance;
        
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        autoController = new HolonomicDriveController(xController, yController, rotController); 
        autoController.setTolerance(new Pose2d(.5,.5,Rotation2d.fromDegrees(10)));

        addRequirements(drive);
    }
    @Override
    public void initialize() {
        autoStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        Trajectory.State goal = path.sample(getAutoTime());
        Rotation2d target = desiredAutoHeading;
        SmartDashboard.putNumber("desired rotation", target.getDegrees());
        SmartDashboard.putNumber("heading error",Math.abs(target.rotateBy(Odometry.getInstance().getOdometry().getRotation()).getDegrees()));
        SmartDashboard.putNumber("time elapsed", getAutoTime());

        ChassisSpeeds desiredSpeeds = autoController.calculate(Odometry.getInstance().getOdometry(), goal, target);

        drive.driveFromChassis(desiredSpeeds);

        SmartDashboard.putNumber("total time", currTrajectory.getTotalTimeSeconds());

        if(autoController.atReference()&&getAutoTime()>= currTrajectory.getTotalTimeSeconds()){
            setDriveState(DriveState.DONE);
        }
    }

    @Override
    public boolean isFinished() {
        
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }
}
