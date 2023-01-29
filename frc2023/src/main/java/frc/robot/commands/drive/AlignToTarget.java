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
    private boolean atGoal;
    private double deltaSpeed;

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
        atGoal= false;
        controller = new ProfiledPIDController(.3, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , 2*Math.PI));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Units.degreesToRadians(3));

    }

    @Override
    public void execute() {
        Rotation2d onTarget = Rotation2d.fromDegrees(0);
        double error = onTarget.rotateBy(vision.getLimeYaw()).getRadians();


        if(Math.abs(error)>Units.degreesToRadians(3))
            deltaSpeed = controller.calculate(error);
        else{
            deltaSpeed =0;
            atGoal = true;
        }

        driveFromChassis(new ChassisSpeeds(0, 0, deltaSpeed*DriveConstants.MAX_TELE_ANGULAR_VELOCITY));

        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
        SmartDashboard.putBoolean("On target", controller.atGoal());
    }

    @Override
    public boolean isFinished() {
        return atGoal;
    }
    @Override
    public void end(boolean interrupted) {
        driveFromChassis(new ChassisSpeeds());
    }
}
