package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AutoDock extends CommandBase{
    Drive drive;
    double lastKnownPitch;
    double driveSpeed;
    ChassisSpeeds chassisSpeeds;

    public AutoDock(Drive drive){
        this.drive = drive;
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);

        lastKnownPitch = drive.getDrivePitch();
        driveSpeed = 0;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        lastKnownPitch = drive.getDrivePitch();

    }

    @Override
    public void execute() {
        if(Math.abs(drive.getDrivePitch() - lastKnownPitch)  > 3){
            driveSpeed = 0;
            drive.lockModules();
        }
        else{
            driveSpeed = 0.05;
            chassisSpeeds = new ChassisSpeeds(driveSpeed * DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY, 0, 0);
            driveFromChassis(chassisSpeeds);
        }

        driveFromChassis(chassisSpeeds);

        lastKnownPitch = drive.getDrivePitch();
    }

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }
}
