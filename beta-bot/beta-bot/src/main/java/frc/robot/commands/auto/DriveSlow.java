package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AltConstants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class DriveSlow extends CommandBase {
    private PIDController controller = new PIDController(.75, 0, 0);

    private Pose2d initial;
    private double distance;
    private Drive drive;
    private Tracker tracker;
    private double deltaSpeed;
    private boolean overridePosition;
    private Pose2d newPosition;
    private Timer timer = new Timer();

      public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }
    public DriveSlow(double distance,Drive drive,Tracker tracker){
        this.distance = distance;
        this.drive = drive;
        this.tracker = tracker;

        controller.setTolerance(.05);
        overridePosition = false;

        addRequirements(drive,tracker);
    }

    public DriveSlow(double distance, Drive drive, Tracker tracker,Pose2d newPosition){
        this.distance = distance;
        this.drive = drive;
        this.tracker = tracker;

        controller.setTolerance(.05);
        overridePosition = true;
        this.newPosition = newPosition;

        addRequirements(drive,tracker);
    }

    @Override
    public void initialize() {
        timer.reset();
        initial = tracker.getPose();
        controller.reset();
        controller.setSetpoint(initial.getX()+distance);
        timer.start();
    }

    @Override
    public void execute() {
        driveFromChassis(new ChassisSpeeds(1, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return timer.get()>.3;
    }

    @Override
    public void end(boolean interrupted) {
    }

}