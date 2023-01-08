package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAuto  extends CommandBase{
    
    @Override
    public void execute() {
        Trajectory.State goal = currTrajectory.sample(getAutoTime());
        Rotation2d target = desiredAutoHeading;
        SmartDashboard.putNumber("desired rotation", target.getDegrees());
        SmartDashboard.putNumber("heading error",Math.abs(target.rotateBy(Odometry.getInstance().getOdometry().getRotation()).getDegrees()));
        SmartDashboard.putNumber("time elapsed", getAutoTime());

        ChassisSpeeds desiredSpeeds = autoController.calculate(Odometry.getInstance().getOdometry(), goal, target);

        driveFromChassis(desiredSpeeds);

        SmartDashboard.putNumber("total time", currTrajectory.getTotalTimeSeconds());

        if(autoController.atReference()&&getAutoTime()>= currTrajectory.getTotalTimeSeconds()){
            setDriveState(DriveState.DONE);
        }
    }
}
