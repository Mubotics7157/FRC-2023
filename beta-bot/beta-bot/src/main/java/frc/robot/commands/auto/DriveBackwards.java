package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AltConstants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class DriveBackwards extends CommandBase {
    private PIDController controller = new PIDController(2, 0, 0);

    private Pose2d initial;
    private double distance;
    private Drive drive;
    private Tracker tracker;
    private double deltaSpeed;
    private boolean overridePosition;
    private Pose2d newPosition;

      public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }
    public DriveBackwards(double distance,Drive drive,Tracker tracker){
        this.distance = distance;
        this.drive = drive;
        this.tracker = tracker;

        controller.setTolerance(.05);
        overridePosition = false;

        addRequirements(drive,tracker);
    }

    public DriveBackwards(double distance, Drive drive, Tracker tracker,Pose2d newPosition){
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
        initial = tracker.getPose();
        controller.reset();
        controller.setSetpoint(initial.getX()+distance);
    }

    @Override
    public void execute() {
        deltaSpeed = controller.calculate(tracker.getPose().getX());
        driveFromChassis(new ChassisSpeeds(deltaSpeed*frc.robot.Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY, 0, 0));
        SmartDashboard.putNumber("Drive Distance Setpoint", controller.getSetpoint());
        SmartDashboard.putBoolean("Drive At Setpoint", controller.atSetpoint());
        SmartDashboard.putNumber("Drive Distance Error", controller.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if(overridePosition)
            tracker.setPose(new Pose2d(newPosition.getTranslation(), tracker.getPose().getRotation()));
    }

}