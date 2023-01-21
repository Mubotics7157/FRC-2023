package frc.robot.commands.drive;

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
import frc.robot.subsystems.VisionManager;

public class AlignToTarget extends CommandBase{
    private Drive drive;
    private VisionManager vision;
    private ProfiledPIDController controller;

    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    public AlignToTarget(Drive dInstance, VisionManager vInstance){
        drive = dInstance;
        vision = vInstance;
        addRequirements(drive);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        controller = new ProfiledPIDController(.3, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , Math.PI));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void execute() {
        controller.setP(SmartDashboard.getNumber("Align kP", 0));
        Rotation2d onTarget = Rotation2d.fromDegrees(0);
        double error = onTarget.rotateBy(vision.getTargetYaw()).getRadians();

        double deltaSpeed = controller.calculate(error);

        driveFromChassis(new ChassisSpeeds(0, 0, deltaSpeed*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY));

        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal() || !vision.hasTargets();
    }
    @Override
    public void end(boolean interrupted) {
        driveFromChassis(new ChassisSpeeds());
    }
}
